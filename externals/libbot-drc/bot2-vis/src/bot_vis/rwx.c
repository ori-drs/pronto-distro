#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <errno.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <GL/glx.h>

#include <glib.h>
#include <math.h>

#include <bot_core/bot_core.h>

#include "tokenize.h"
#include "rwx.h"

static void 
parse_error(tokenize_t *t, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    printf("\n");
    vprintf(fmt, ap);
    printf("\n");

    printf("%s : %i\n", t->path, t->line);
    printf("%s", t->line_buffer);
    for (int i = 0; i < t->column; i++) {
        if (isspace(t->line_buffer[i]))
            printf("%c", t->line_buffer[i]);
        else
            printf(" ");
    }
    printf("^\n");

    va_end(ap);
    _exit(0);
}

static int 
parse_int(tokenize_t *t, int *d, int radix)
{
    char *endptr = NULL;
    *d = strtol(t->token, &endptr, radix);
    if( t->token == endptr ) {
        parse_error(t, "expected integer!");
    }
    tokenize_next(t);
    return 0;
}

static int 
parse_double(tokenize_t *t, double *f) 
{
    char *endptr = NULL;
    *f = strtod(t->token, &endptr);
    if( t->token == endptr ) {
        parse_error(t, "expected float!");
    }
    tokenize_next(t);
    return 0;
}

static void 
parse_require(tokenize_t *t, char *tok)
{
    int res = tokenize_next(t);
    if (res == EOF || strcmp(t->token, tok)) 
        parse_error(t, "expected token %s", tok);
}

static int 
parse_vertex( tokenize_t *tok, BotRwxVertex *v )
{
    tokenize_next( tok );

    parse_double( tok, &v->pos[0] );
    parse_double( tok, &v->pos[1] );
    parse_double( tok, &v->pos[2] );

    if (!strcmp (tok->token, "UV")) {
        // TODO
        tokenize_next (tok);
        double uv;
        parse_double (tok, &uv);
        parse_double (tok, &uv);
    }

    if( tok->token[0] != '#' ) {
        parse_error( tok, "expected vertex id prefixed with '#'!" );
    }
    while (tok->token[1] == 0) {
        tokenize_next (tok);
        if (tok->token[0] != '!')
            break;
        tokenize_next (tok);
        while (tok->token[0] != '#')
            tokenize_next (tok);
    }
    char *endptr;
    v->id = strtol( tok->token + 1, &endptr, 10 );
    if( tok->token+1 == endptr ) {
        parse_error( tok, "expected numerical vertex id!" );
    }
    return 0;
}

static int 
parse_triangle( tokenize_t *tok, BotRwxTriangle *t )
{
    tokenize_next( tok );

    parse_int( tok, &t->vertices[0], 10 );
    parse_int( tok, &t->vertices[1], 10 );
    parse_int( tok, &t->vertices[2], 10 );
    return 0;
}

static int 
parse_clump( BotRwxClump *clump, tokenize_t *tok )
{
    // get clump name
    parse_require( tok, "#Layer" );
    parse_require( tok, ":" );
    tokenize_next( tok );
    clump->name = strdup( tok->token );

    // defaults
    clump->color[0] = clump->color[1] = clump->color[2] = 0;
    clump->opacity = 1;
    clump->ambient = 1;
    clump->diffuse = 1;
    clump->specular = 0;

    int nvbuf = 20000;
    BotRwxVertex *vbuf = (BotRwxVertex*)calloc(1, nvbuf*sizeof(BotRwxVertex));

    // clump color
    tokenize_next( tok );
    while( 0 != strcmp( tok->token, "#texbegin" ) ) {

        if( ! strcmp( tok->token, "Color" ) ) {
            tokenize_next( tok );
            parse_double( tok, &clump->color[0] );
            parse_double( tok, &clump->color[1] );
            parse_double( tok, &clump->color[2] );
        } else if( ! strcmp( tok->token, "Surface" ) ) {
            tokenize_next( tok );
//            parse_double( tok, &clump->surface[0] );
//            parse_double( tok, &clump->surface[1] );
//            parse_double( tok, &clump->surface[2] );
            parse_double( tok, &clump->ambient );
            parse_double( tok, &clump->diffuse );
            parse_double( tok, &clump->specular );
        } else if( ! strcmp( tok->token, "Diffuse" ) ) {
            tokenize_next( tok );
            parse_double( tok, &clump->diffuse );
        } else if( ! strcmp( tok->token, "Specular" ) ) {
            tokenize_next( tok );
            parse_double( tok, &clump->specular );
        } else if( ! strcmp( tok->token, "Opacity" ) ) {
            tokenize_next( tok );
            parse_double( tok, &clump->opacity );
        } else {
            tokenize_next( tok );
        }
    }

    // expect the clump name
    parse_require( tok, clump->name );

    // start with a single vertex that should never be used.
    clump->nvertices = 1;

    // parse the list of vertices
    while( tokenize_next( tok ) != EOF ) {
        if( ! strcmp( tok->token, "Vertex" ) ) {
            parse_vertex( tok, vbuf + clump->nvertices );

            if( vbuf[clump->nvertices].id != clump->nvertices ) {
                parse_error( tok, "expected vertex %d (got %d)!\n", 
                        clump->nvertices, vbuf[clump->nvertices].id );
            }
            
            clump->nvertices++;

            if( clump->nvertices >= nvbuf ) {
                nvbuf += 10000;
                vbuf = (BotRwxVertex*)realloc(vbuf, nvbuf*sizeof(BotRwxVertex));
            }
        } else if( ! strcmp( tok->token, "#texend" ) ) {
            // expect the clump name
            parse_require( tok, clump->name );
            break;
        } else {
            parse_error( tok, "expected Vertex or #texend token!" );
        }
    }

    clump->vertices = 
        (BotRwxVertex*)malloc(clump->nvertices*sizeof(BotRwxVertex));
    memcpy( clump->vertices, vbuf, clump->nvertices*sizeof(BotRwxVertex) );
    free( vbuf );

    int ntbuf = 20000;
    BotRwxTriangle *tbuf = 
        (BotRwxTriangle*)calloc(1, ntbuf*sizeof(BotRwxTriangle));

    // parse the list of triangles
    tokenize_next( tok );
    while( 1 ) {
//    while( tokenize_next( tok ) != EOF ) {
        if( ! strcmp( tok->token, "Triangle" ) ) {
            parse_triangle( tok, tbuf + clump->ntriangles );
            clump->ntriangles++;

            if( clump->ntriangles >= ntbuf ) {
                ntbuf += 10000;
                tbuf = (BotRwxTriangle*)realloc(tbuf, 
                        ntbuf*sizeof(BotRwxTriangle));
            }
        } else if( ! strcmp( tok->token, "ClumpEnd" ) ) {
            break;
        }
    }

    clump->triangles = 
        (BotRwxTriangle*)malloc(clump->ntriangles*sizeof(BotRwxTriangle));
    memcpy( clump->triangles, tbuf, clump->ntriangles*sizeof(BotRwxTriangle) );
    free( tbuf );

    return 0;
}

BotRwxModel* 
bot_rwx_model_create( const char *fname )
{
    BotRwxModel *model = calloc(1, sizeof(BotRwxModel)); 

    tokenize_t *tok = tokenize_create( fname );
    int status;

    if (NULL==tok) {
        perror(fname);
        free(model);
        return NULL;
    }

    parse_require( tok, "ModelBegin" );
    parse_require( tok, "ClumpBegin" );

    while (tokenize_next(tok)!=EOF) {
        if( 0 == strcmp( tok->token, "ClumpBegin" ) ) {
            BotRwxClump *clump = (BotRwxClump*)calloc(1, sizeof(BotRwxClump));
            model->clumps = g_list_append( model->clumps, clump );

            status = parse_clump( clump, tok );
            if( 0 != status ) {
                tokenize_destroy(tok);
                bot_rwx_model_destroy(model);
                return NULL;
            }
        } else if( 0 == strcmp( tok->token, "ClumpEnd" ) ) {
            parse_require( tok, "ModelEnd" );
            break;
        }
    }

    tokenize_destroy( tok );
    return model;
}

void
bot_rwx_model_destroy( BotRwxModel *model )
{
    // cleanup clumps
    GList *citer;
    for( citer = model->clumps; citer != NULL; citer = citer->next ) {
        BotRwxClump *clump = (BotRwxClump*)citer->data;
        free( clump->vertices );
        free( clump->triangles );
        free( clump->name );
        free( clump );
    }
    g_list_free( model->clumps );
    free( model );
}

void
bot_rwx_model_get_extrema (BotRwxModel * model, double minv[3], double maxv[3])
{
    GList *citer;
    int i;
    for (i = 0; i < 3; i++) {
        minv[i] = INFINITY;
        maxv[i] = -INFINITY;
    }

    for (citer = model->clumps; citer != NULL; citer = citer->next) {
        BotRwxClump *clump = (BotRwxClump*)citer->data;
        for (i = 1; i < clump->nvertices; i++) {
            BotRwxVertex * v = clump->vertices + i;
            int j;
            for (j = 0; j < 3; j++) {
                if (v->pos[j] < minv[j])
                    minv[j] = v->pos[j];
                if (v->pos[j] > maxv[j])
                    maxv[j] = v->pos[j];
            }
        }
    }
}

// XXX why this function?
static inline void
_matrix_vector_multiply_4x4_3d2 (const double m[16], const double v[3],
                               double result[3]) {
    
    result[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2] + m[3];
    result[1] = m[4]*v[0] + m[5]*v[1] + m[6]*v[2] + m[7];
    result[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11];
    
    double result3   = m[12]*v[0] + m[13]*v[1] + m[14]*v[2] + m[15];
    if (result3) {
        result[0]/=result3;
        result[1]/=result3;
        result[2]/=result3;
    }
}    

void
bot_rwx_model_apply_transform (BotRwxModel * model, double m[16])
{
    GList *citer;

    for (citer = model->clumps; citer != NULL; citer = citer->next) {
        BotRwxClump *clump = (BotRwxClump*)citer->data;
        for (int i = 0; i < clump->nvertices; i++) {
            BotRwxVertex * v = clump->vertices + i;
            double old_pos[3]={v->pos[0],v->pos[1],v->pos[2]};
            _matrix_vector_multiply_4x4_3d2(m,old_pos,v->pos);
        }
    }
}


void
bot_rwx_model_gl_draw( BotRwxModel *model ) 
{
    GList *citer;

    double a[3], b[3];
    double n[3];

    float color[4];
#if 0
    float minv[3] = {INFINITY, INFINITY, INFINITY };
    float maxv[3] = {-INFINITY, -INFINITY, -INFINITY };
#endif

    for( citer = model->clumps; citer != NULL; citer = citer->next ) {
        BotRwxClump *clump = (BotRwxClump*)citer->data;
        int i;

        /*glColor4f( clump->color[0], clump->color[1], clump->color[2],
          clump->opacity ); */

        // ambient
        if (clump->ambient < .5)
            clump->ambient = 0.5;

        color[0] = clump->color[0] * clump->ambient;
        color[1] = clump->color[1] * clump->ambient;
        color[2] = clump->color[2] * clump->ambient;
        color[3] = 1;
        glMaterialfv( GL_FRONT, GL_AMBIENT, color );

        // diffuse
        //	clump->diffuse = 1.0;
        color[0] = clump->color[0] * clump->diffuse;
        color[1] = clump->color[1] * clump->diffuse;
        color[2] = clump->color[2] * clump->diffuse;
        color[3] = clump->opacity;
        glMaterialfv( GL_FRONT, GL_DIFFUSE, color );

        // emission
        color[0] = color[1] = color[2] = 0;
        color[3] = 1;
        glMaterialfv( GL_FRONT, GL_EMISSION, color );

        // specular
        color[0] = color[1] = color[2] = clump->specular;
        color[3] = 1;
        glMaterialfv( GL_FRONT, GL_SPECULAR, color );

        // XXX hard-code shininess for lack of information
        glMateriali( GL_FRONT, GL_SHININESS, 20 );

        glBegin( GL_TRIANGLES );

        // For every single vertex, average out the normal vectors for every
        // triangle that the vertex participates in.  Set that averaged vector
        // as the normal vector for that vertex.  This results in a much
        // smoother rendered model than the simple way (which is to just have
        // a single normal vector for all three vertices of a triangle when
        // the triangle is drawn).
        int *vertex_counts = (int*)calloc(1,clump->nvertices*sizeof(int));
        double *normals = (double*)calloc(1,clump->nvertices*sizeof(double)*3);

#if 0
        for (i = 1; i < clump->nvertices; i++) {
            BotRwxVertex * v = clump->vertices + i;
            int j;
            for (j = 0; j < 3; j++) {
                if (v->pos[j] < minv[j])
                    minv[j] = v->pos[j];
                if (v->pos[j] > maxv[j])
                    maxv[j] = v->pos[j];
            }
        }
#endif

        // account for the normal vector of every triangle.
        for( i=1; i<clump->ntriangles; i++ ) {
            // find the vertex indices
            int vid1, vid2, vid3;
            vid1 = clump->triangles[i].vertices[0];
            vid2 = clump->triangles[i].vertices[1];
            vid3 = clump->triangles[i].vertices[2];
            // load the vertices
            BotRwxVertex *v1, *v2, *v3;
            v1 = &clump->vertices[vid1];
            v2 = &clump->vertices[vid2];
            v3 = &clump->vertices[vid3];
            // compute and average in the normal
            bot_vector_subtract_3d( v2->pos, v1->pos, a );
            bot_vector_subtract_3d( v3->pos, v1->pos, b );
            bot_vector_cross_3d( a, b, n );
            double nmag = sqrt( n[0]*n[0] + n[1]*n[1] + n[2]*n[2] );
            n[0] /= nmag;
            n[1] /= nmag;
            n[2] /= nmag;

            vertex_counts[vid1]++;
            vertex_counts[vid2]++;
            vertex_counts[vid3]++;

            normals[vid1*3 + 0] += n[0];
            normals[vid1*3 + 1] += n[1];
            normals[vid1*3 + 2] += n[2];
            normals[vid2*3 + 0] += n[0];
            normals[vid2*3 + 1] += n[1];
            normals[vid2*3 + 2] += n[2];
            normals[vid3*3 + 0] += n[0];
            normals[vid3*3 + 1] += n[1];
            normals[vid3*3 + 2] += n[2];
        }

        // scale the resulting vectors to be of unit length
        for( i=1; i<clump->nvertices; i++ ) {
            normals[i*3 + 0] /= vertex_counts[i];
            normals[i*3 + 1] /= vertex_counts[i];
            normals[i*3 + 2] /= vertex_counts[i];

            // re-normalize, because averaging out unit-length vectors doesn't
            // always result in a unit-length vector.
            double *n = normals + i*3;
            double nmag = sqrt( n[0]*n[0] + n[1]*n[1] + n[2]*n[2] );
            n[0] /= nmag;
            n[1] /= nmag;
            n[2] /= nmag;
        }

        free( vertex_counts );

        for( i=0; i<clump->ntriangles; i++ ) {
            // find the vertex indices
            int vid1, vid2, vid3;
            vid1 = clump->triangles[i].vertices[0];
            vid2 = clump->triangles[i].vertices[1];
            vid3 = clump->triangles[i].vertices[2];

            // load the vertices
            BotRwxVertex *v1, *v2, *v3;
            v1 = &clump->vertices[vid1];
            v2 = &clump->vertices[vid2];
            v3 = &clump->vertices[vid3];

#if 0
            // compute and set the normal
            vector_subtract_3d( v2->pos, v1->pos, a );
            vector_subtract_3d( v3->pos, v1->pos, b );
            vector_cross_3d( a, b, n );
            double nmag = sqrt( n[0]*n[0] + n[1]*n[1] + n[2]*n[2] );
            n[0] /= nmag;
            n[1] /= nmag;
            n[2] /= nmag;
            glNormal3d( n[0], n[1], n[2] );
#endif

            // render the triangle
            glNormal3d( normals[vid1*3+ 0], 
                    normals[vid1*3 + 1],
                    normals[vid1*3 + 2]);
            glVertex3f( v1->pos[0], v1->pos[1], v1->pos[2] );
            glNormal3d( normals[vid2*3+ 0], 
                    normals[vid2*3 + 1],
                    normals[vid2*3 + 2]);
            glVertex3f( v2->pos[0], v2->pos[1], v2->pos[2] );
            glNormal3d( normals[vid3*3 + 0], 
                    normals[vid3*3 + 1],
                    normals[vid3*3 + 2]);
            glVertex3f( v3->pos[0], v3->pos[1], v3->pos[2] );
        }

        free( normals );
        glEnd();
    }

#if 0
    printf ("max-min %f %f %f\n", maxv[0] - minv[0], maxv[1] - minv[1],
            maxv[2] - minv[2]);
    printf ("min %f %f %f\n", minv[0], minv[1], minv[2]);
#endif
    
#if 0
    glLineWidth( 4.0 );
    glBegin( GL_LINES );
    glColor4f( 1, 1, 1, 0.5 );

    for( citer = model->clumps; citer != NULL; citer = citer->next ) {
        BotRwxClump *clump = (BotRwxClump*)citer->data;
        int i;
        for( i=0; i<clump->ntriangles; i++ ) {
            // find the vertex indices
            int vid1, vid2, vid3;
            vid1 = clump->triangles[i].vertices[0];
            vid2 = clump->triangles[i].vertices[1];
            vid3 = clump->triangles[i].vertices[2];

            // load the vertices
            BotRwxVertex *v1, *v2, *v3;
            v1 = &clump->vertices[vid1];
            v2 = &clump->vertices[vid2];
            v3 = &clump->vertices[vid3];

            // compute and set the normal
            vector_subtract_3d( v2->pos, v1->pos, a );
            vector_subtract_3d( v3->pos, v1->pos, b );
            vector_cross_3d( a, b, n );
            double nmag = sqrt( n[0]*n[0] + n[1]*n[1] + n[2]*n[2] );
            n[0] /= nmag;
            n[1] /= nmag;
            n[2] /= nmag;

            n[0] *= 5;
            n[1] *= 5;
            n[2] *= 5;

            // midpoint of the triangle
            double m[3];
            m[0] = (v1->pos[0] + v2->pos[0] + v3->pos[0]) / 3;
            m[1] = (v1->pos[1] + v2->pos[1] + v3->pos[1]) / 3;
            m[2] = (v1->pos[2] + v2->pos[2] + v3->pos[2]) / 3;

            // render the normal vector
            glVertex3f( m[0], m[1], m[2] );
            glVertex3f( m[0] + n[0], m[1] + n[1], m[2] + n[2] );
        }
    }
    glEnd();
#endif
}
