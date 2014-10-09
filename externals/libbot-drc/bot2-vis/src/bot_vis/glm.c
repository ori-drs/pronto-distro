/*    
      glm.c
      Nate Robins, 1997, 2000
      nate@pobox.com, http://www.pobox.com/~nate
 
      Wavefront OBJ model file format reader/writer/manipulator.

      Includes routines for generating smooth normals with
      preservation of edges, welding redundant vertices & texture
      coordinate generation (spheremap and planar projections) + more.
  
      changes by F. Devernay:
      - warning/error functions in glm_util.c
      - added glmStrStrip function to handle filenames with spaces
      - handle material-by-face if there is more than one usemtl in a group
      - removed "static" from glmDraw variables (so that the code is reentrant)
      - write obj with material-by-face
      - added blending support, from GLM_AVL http://www.avl.iu.edu/projects/GLM_AVL/

      TODO:
      - allow CR, CRLF, or LF as end-of-line in input obj and mtl
      - glmVertexNormals(): have an option to add normals only where undefined
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define MATERIAL_BY_FACE

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "glm.h"
/*
#define DEBUG
#define GLDEBUG
*/
#include "glmint.h"

#define T(x) (model->triangles[(x)])


/* _GLMnode: general purpose node */
typedef struct _GLMnode {
    GLuint         index;
    GLboolean      averaged;
    struct _GLMnode* next;
} GLMnode;



/* glmMax: returns the maximum of two floats */
static GLfloat
glmMax(GLfloat a, GLfloat b) 
{
    if (b > a)
        return b;
    return a;
}

/* glmAbs: returns the absolute value of a float */
static GLfloat
glmAbs(GLfloat f)
{
    if (f < 0)
        return -f;
    return f;
}

/* glmDot: compute the dot product of two vectors
 *
 * u - array of 3 GLfloats (GLfloat u[3])
 * v - array of 3 GLfloats (GLfloat v[3])
 */
static GLfloat
glmDot(GLfloat* u, GLfloat* v)
{
    assert(u); assert(v);
    
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

/* glmCross: compute the cross product of two vectors
 *
 * u - array of 3 GLfloats (GLfloat u[3])
 * v - array of 3 GLfloats (GLfloat v[3])
 * n - array of 3 GLfloats (GLfloat n[3]) to return the cross product in
 */
static GLvoid
glmCross(GLfloat* u, GLfloat* v, GLfloat* n)
{
    assert(u); assert(v); assert(n);
    
    n[0] = u[1]*v[2] - u[2]*v[1];
    n[1] = u[2]*v[0] - u[0]*v[2];
    n[2] = u[0]*v[1] - u[1]*v[0];
}

/* glmNormalize: normalize a vector
 *
 * v - array of 3 GLfloats (GLfloat v[3]) to be normalized
 */
static GLvoid
glmNormalize(GLfloat* v)
{
    GLfloat l;
    
    assert(v);
    
    l = (GLfloat)sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    v[0] /= l;
    v[1] /= l;
    v[2] /= l;
}

/* glmEqual: compares two vectors and returns GL_TRUE if they are
 * equal (within a certain threshold) or GL_FALSE if not. An epsilon
 * that works fairly well is 0.000001.
 *
 * u - array of 3 GLfloats (GLfloat u[3])
 * v - array of 3 GLfloats (GLfloat v[3]) 
 */
static GLboolean
glmEqual(GLfloat* u, GLfloat* v, GLfloat epsilon)
{
    if (glmAbs(u[0] - v[0]) < epsilon &&
        glmAbs(u[1] - v[1]) < epsilon &&
        glmAbs(u[2] - v[2]) < epsilon) 
	{
	    return GL_TRUE;
	}
    return GL_FALSE;
}

/* glmWeldVectors: eliminate (weld) vectors that are within an
 * epsilon of each other.
 *
 * vectors     - array of GLfloat[3]'s to be welded
 * numvectors - number of GLfloat[3]'s in vectors
 * epsilon     - maximum difference between vectors 
 *
 */
static GLfloat*
glmWeldVectors(GLfloat* vectors, GLuint* numvectors, GLfloat epsilon)
{
    GLfloat* copies;
    GLuint   copied;
    GLuint   i, j;
    
    copies = (GLfloat*)malloc(sizeof(GLfloat) * 3 * (*numvectors + 1));
    memcpy(copies, vectors, (sizeof(GLfloat) * 3 * (*numvectors + 1)));
    
    copied = 1;
    for (i = 1; i <= *numvectors; i++) {
        for (j = 1; j <= copied; j++) {
            if (glmEqual(&vectors[3 * i], &copies[3 * j], epsilon)) {
                goto duplicate;
            }
        }
        
        /* must not be any duplicates -- add to the copies array */
        copies[3 * copied + 0] = vectors[3 * i + 0];
        copies[3 * copied + 1] = vectors[3 * i + 1];
        copies[3 * copied + 2] = vectors[3 * i + 2];
        j = copied;             /* pass this along for below */
        copied++;
        
      duplicate:
/* set the first component of this vector to point at the correct
   index into the new copies array */
        vectors[3 * i + 0] = (GLfloat)j;
    }
    
    *numvectors = copied-1;
    return copies;
}



/* glmFindGroup: Find a group in the model */
static GLMgroup*
glmFindGroup(GLMmodel* model, char* name)
{
    GLMgroup* group;
    
    assert(model);
    
    group = model->groups;
    while(group) {
        if (!strcmp(name, group->name))
            break;
        group = group->next;
    }
    
    return group;
}

/* glmAddGroup: Add a group to the model */
static GLMgroup*
glmAddGroup(GLMmodel* model, char* name)
{
    GLMgroup* group;
    
    group = glmFindGroup(model, name);
    if (!group) {
        group = (GLMgroup*)malloc(sizeof(GLMgroup));
        group->name = __glmStrdup(name);
        group->material = 0;
        group->numtriangles = 0;
        group->triangles = NULL;
        group->next = model->groups;
        model->groups = group;
        model->numgroups++;
    }
    
    return group;
}

/* glmFindGroup: Find a material in the model */
static GLuint
glmFindMaterial(GLMmodel* model, char* name)
{
    GLuint i;
    
    assert(name != NULL);
    /* XXX doing a linear search on a string key'd list is pretty lame,
       but it works and is fast enough for now. */
    for (i = 0; i < model->nummaterials; i++) {
	assert(model->materials[i].name != NULL);
        if (!strcmp(model->materials[i].name, name))
            goto found;
    }
    
    /* didn't find the name, so print a warning and return the default
       material (0). */
    __glmWarning("glmFindMaterial():  can't find material \"%s\".", name);
    i = 0;
    
  found:
    return i;
}





/* glmFindTexture: Find a texture in the model */
static GLuint
glmFindOrAddTexture(GLMmodel* model, const char* name)
{
    GLuint i;
    char *dir, *filename;
    float width, height;

    /* XXX doing a linear search on a string key'd list is pretty lame,
       but it works and is fast enough for now. */
    for (i = 0; i < model->numtextures; i++) {
        if (!strcmp(model->textures[i].name, name))
            return i;
    }
    
    dir = __glmDirName(model->pathname);
    filename = (char*)malloc(sizeof(char) * (strlen(dir) + strlen(name) + 1));
    strcpy(filename, dir);
    strcat(filename, name);
    free(dir);

    /* didn't find the name, so print a warning and return the default
       texture (0). */
    model->numtextures++;
    model->textures = (GLMtexture*)realloc(model->textures, sizeof(GLMtexture)*model->numtextures);
    model->textures[model->numtextures-1].name = strdup(name);
    model->textures[model->numtextures-1].id =
        glmLoadTexture(filename, GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE, &width, &height);
    model->textures[model->numtextures-1].width = width;
    model->textures[model->numtextures-1].height = height;
    DBG_(__glmWarning("allocated texture %d (id=%d,width=%g,height=%g)",model->numtextures-1, model->textures[model->numtextures-1].id, width, height));

    free(filename);

    return model->numtextures-1;
}

/* glmReadMTL: read a wavefront material library file
 *
 * model - properly initialized GLMmodel structure
 * name  - name of the material library
 */
static GLvoid
glmReadMTL(GLMmodel* model, char* name)
{
    FILE* file;
    char* dir;
    char* filename;
    char* t_filename;
    char    buf[128];
    GLuint nummaterials, i;
    
    dir = __glmDirName(model->pathname);
    filename = (char*)malloc(sizeof(char) * (strlen(dir) + strlen(name) + 1));
    strcpy(filename, dir);
    strcat(filename, name);
    
    file = fopen(filename, "r");
    if (!file) {
        __glmFatalError( "glmReadMTL() failed: can't open material file \"%s\".",
			 filename);
    }
    free(filename);
    
    /* count the number of materials in the file */
    nummaterials = 1;
    char *rem;
    int remi;
    while(fscanf(file, "%s", buf) != EOF) {
        switch(buf[0]) {
        case '#':               /* comment */
            /* eat up rest of line */
            rem = fgets(buf, sizeof(buf), file);
            break;
        case 'n':               /* newmtl */
	    if(strncmp(buf, "newmtl", 6) != 0)
		__glmFatalError("glmReadMTL: Got \"%s\" instead of \"newmtl\" in file \"%s\"", buf, filename);
            rem = fgets(buf, sizeof(buf), file);
            nummaterials++;
            sscanf(buf, "%s %s", buf, buf);
            break;
        default:
            /* eat up rest of line */
            rem = fgets(buf, sizeof(buf), file);
            break;
        }
    }
    
    rewind(file);
    
    model->materials = (GLMmaterial*)malloc(sizeof(GLMmaterial) * nummaterials);
    model->nummaterials = nummaterials;
    
    /* set the default material */
    for (i = 0; i < nummaterials; i++) {
        model->materials[i].name = NULL;
        model->materials[i].shininess = 65.0;
        model->materials[i].diffuse[0] = 0.8;
        model->materials[i].diffuse[1] = 0.8;
        model->materials[i].diffuse[2] = 0.8;
        model->materials[i].diffuse[3] = 1.0;
        model->materials[i].ambient[0] = 0.2;
        model->materials[i].ambient[1] = 0.2;
        model->materials[i].ambient[2] = 0.2;
        model->materials[i].ambient[3] = 1.0;
        model->materials[i].specular[0] = 0.0;
        model->materials[i].specular[1] = 0.0;
        model->materials[i].specular[2] = 0.0;
        model->materials[i].specular[3] = 1.0;
        model->materials[i].map_diffuse = -1;
    }
    model->materials[0].name = __glmStrdup("default");
    
    /* now, read in the data */
    nummaterials = 0;
    while(fscanf(file, "%s", buf) != EOF) {
        switch(buf[0]) {
        case '#':               /* comment */
            /* eat up rest of line */
            rem = fgets(buf, sizeof(buf), file);
            break;
        case 'n':               /* newmtl */
#if 0
            __glmWarning("name=%s; Ns=%g; Ka=%g,%g,%g; Kd=%g,%g,%g; Ks=%g,%g,%g",
                         model->materials[nummaterials].name,
                         model->materials[nummaterials].shininess/128.0*GLM_MAX_SHININESS,
                         model->materials[nummaterials].ambient[0],
                         model->materials[nummaterials].ambient[1],
                         model->materials[nummaterials].ambient[2],
                         model->materials[nummaterials].diffuse[0],
                         model->materials[nummaterials].diffuse[1],
                         model->materials[nummaterials].diffuse[2],
                         model->materials[nummaterials].specular[0],
                         model->materials[nummaterials].specular[1],
                         model->materials[nummaterials].specular[2]);
#endif
            if(strncmp(buf, "newmtl", 6) != 0)
                __glmFatalError("glmReadMTL: Got \"%s\" instead of \"newmtl\" in file \"%s\"", buf, filename);
            rem = fgets(buf, sizeof(buf), file);
            sscanf(buf, "%s %s", buf, buf);
            nummaterials++;
            model->materials[nummaterials].name = __glmStrdup(buf);
            break;
        case 'N':
            switch(buf[1]) {
            case 's':
                remi = fscanf(file, "%f", &model->materials[nummaterials].shininess);
                /* wavefront shininess is from [0, 1000], so scale for OpenGL */
                model->materials[nummaterials].shininess /= GLM_MAX_SHININESS;
                model->materials[nummaterials].shininess *= 128.0;
                break;
            case 'i':
                /* Refraction index.  Values range from 1 upwards. A value
                   of 1 will cause no refraction. A higher value implies
                   refraction. */
                __glmWarning("refraction index ignored");
                rem = fgets(buf, sizeof(buf), file);
                break;
            default:
                __glmWarning("glmReadMTL: Command \"%s\" ignored", buf);
                rem = fgets(buf, sizeof(buf), file);
                break;
            }
            break;
        case 'K':
            switch(buf[1]) {
            case 'd':
                remi = fscanf(file, "%f %f %f",
                       &model->materials[nummaterials].diffuse[0],
                       &model->materials[nummaterials].diffuse[1],
                       &model->materials[nummaterials].diffuse[2]);
                break;
            case 's':
                remi = fscanf(file, "%f %f %f",
                       &model->materials[nummaterials].specular[0],
                       &model->materials[nummaterials].specular[1],
                       &model->materials[nummaterials].specular[2]);
                break;
            case 'a':
                remi = fscanf(file, "%f %f %f",
                       &model->materials[nummaterials].ambient[0],
                       &model->materials[nummaterials].ambient[1],
                       &model->materials[nummaterials].ambient[2]);
                break;
            default:
                __glmWarning("glmReadMTL: Command \"%s\" ignored", buf);
                /* eat up rest of line */
                rem = fgets(buf, sizeof(buf), file);
                break;
            }
            break;
        case 'd':
            /* d = Dissolve factor (pseudo-transparency).
               Values are from 0-1. 0 is completely transparent, 1 is opaque. */
            {
                float alpha;
                remi = fscanf(file, "%f", &alpha);
                model->materials[nummaterials].diffuse[3] = alpha;
            }
            break;
        case 'i':
            if(strncmp(buf, "illum", 5) != 0)
                __glmFatalError("glmReadMTL: Got \"%s\" instead of \"illum\" in file \"%s\"", buf, filename);
            /* illum = (0, 1, or 2) 0 to disable lighting, 1 for
               ambient & diffuse only (specular color set to black), 2
               for full lighting. I've also seen values of 3 and 4 for
               'illum'... when there's a 3 there, there's often a
               'sharpness' attribute, but I didn't find any
               explanation. And I think the 4 illum value is supposed
               to denote two-sided polygons, but I kinda get the
               impression that some people just make stuff up and add
               whatever they want to these files, so there could be
               anything in there ;). */
            {
                int illum;
                remi = fscanf(file, "%d", &illum);
                if(illum != 2)	/* illum=2 is standard lighting */
                    __glmWarning("illum material ignored: illum %d", illum);
            }
            break;
        case 'm':
            /* texture map */
            filename = malloc(FILENAME_MAX);
            rem = fgets(filename, FILENAME_MAX, file);
            t_filename = __glmStrStrip((char*)filename);
            free(filename);
            if(strncmp(buf, "map_Kd", 6) == 0) {
                model->materials[nummaterials].map_diffuse = glmFindOrAddTexture(model, t_filename);
                free(t_filename);
            } else {
                __glmWarning("map %s %s ignored",buf,t_filename);
                free(t_filename);
                rem = fgets(buf, sizeof(buf), file);
            }
            break;
        case 'r':
            /* reflection type and filename (?) */
            rem = fgets(buf, sizeof(buf), file);
            __glmWarning("reflection type ignored: r%s",buf);
            break;
        default:
            /* eat up rest of line */
            rem = fgets(buf, sizeof(buf), file);
            break;
        }
    }
    free(dir);
    fclose(file);
}

/* glmWriteMTL: write a wavefront material library file
 *
 * model   - properly initialized GLMmodel structure
 * modelpath  - pathname of the model being written
 * mtllibname - name of the material library to be written
 */
static GLvoid
glmWriteMTL(GLMmodel* model, char* modelpath, char* mtllibname)
{
    FILE* file;
    char* dir;
    char* filename;
    GLMmaterial* material;
    GLuint i;
    
    dir = __glmDirName(modelpath);
    filename = (char*)malloc(sizeof(char) * (strlen(dir)+strlen(mtllibname)+1));
    strcpy(filename, dir);
    strcat(filename, mtllibname);
    free(dir);
    
    /* open the file */
    file = fopen(filename, "w");
    if (!file) {
        __glmFatalError( "glmWriteMTL() failed: can't open file \"%s\".",
			 filename);
    }
    free(filename);
    
    /* spit out a header */
    fprintf(file, "#  \n");
    fprintf(file, "#  Wavefront MTL generated by GLM library\n");
    fprintf(file, "#  \n");
    fprintf(file, "#  GLM library\n");
    fprintf(file, "#  Nate Robins\n");
    fprintf(file, "#  ndr@pobox.com\n");
    fprintf(file, "#  http://www.pobox.com/~ndr\n");
    fprintf(file, "#  \n\n");
    
    for (i = 0; i < model->nummaterials; i++) {
        material = &model->materials[i];
        fprintf(file, "newmtl %s\n", material->name);
        fprintf(file, "Ka %f %f %f\n", 
		material->ambient[0], material->ambient[1], material->ambient[2]);
        fprintf(file, "Kd %f %f %f\n", 
		material->diffuse[0], material->diffuse[1], material->diffuse[2]);
        fprintf(file, "Ks %f %f %f\n", 
		material->specular[0],material->specular[1],material->specular[2]);
        fprintf(file, "Ns %f\n", material->shininess / 128.0 * GLM_MAX_SHININESS);
        fprintf(file, "\n");
    }
}


/* glmFirstPass: first pass at a Wavefront OBJ file that gets all the
 * statistics of the model (such as #vertices, #normals, etc)
 *
 * model - properly initialized GLMmodel structure
 * file  - (fopen'd) file descriptor 
 */
static GLvoid
glmFirstPass(GLMmodel* model, FILE* file) 
{
    GLuint  numvertices;        /* number of vertices in model */
    GLuint  numnormals;         /* number of normals in model */
    GLuint  numtexcoords;       /* number of texcoords in model */
    GLuint  numtriangles;       /* number of triangles in model */
    GLMgroup* group;            /* current group */
    unsigned    v, n, t;
    char        buf[128];
    char *rem;
    int remi;

    /* make a default group */
    group = glmAddGroup(model, "default");
    
    numvertices = numnormals = numtexcoords = numtriangles = 0;
    while(fscanf(file, "%s", buf) != EOF) {
        switch(buf[0]) {
        case '#':               /* comment */
            /* eat up rest of line */
            rem = fgets(buf, sizeof(buf), file);
            break;
        case 'v':               /* v, vn, vt */
            switch(buf[1]) {
            case '\0':          /* vertex */
                /* eat up rest of line */
                rem = fgets(buf, sizeof(buf), file);
                numvertices++;
                break;
            case 'n':           /* normal */
                /* eat up rest of line */
                rem = fgets(buf, sizeof(buf), file);
                numnormals++;
                break;
            case 't':           /* texcoord */
                /* eat up rest of line */
                rem = fgets(buf, sizeof(buf), file);
                numtexcoords++;
                break;
            default:
                __glmFatalError("glmFirstPass(): Unknown token \"%s\".", buf);
                break;
            }
            break;
	case 'm':
	    if(strncmp(buf, "mtllib", 6) != 0)
		__glmFatalError("glmReadOBJ: Got \"%s\" instead of \"mtllib\"", buf);
	    rem = fgets(buf, sizeof(buf), file);
	    sscanf(buf, "%s %s", buf, buf);
	    model->mtllibname = __glmStrStrip((char*)buf);
	    glmReadMTL(model, model->mtllibname);
	    break;
	case 'u':
	    if(strncmp(buf, "usemtl", 6) != 0)
		__glmFatalError("glmReadOBJ: Got \"%s\" instead of \"usemtl\"", buf);
	    /* eat up rest of line */
	    rem = fgets(buf, sizeof(buf), file);
	    break;
	case 'g':               /* group */
	    /* eat up rest of line */
	    rem = fgets(buf, sizeof(buf), file);
#if SINGLE_STRING_GROUP_NAMES
	    sscanf(buf, "%s", buf);
#else
	    buf[strlen(buf)-1] = '\0';  /* nuke '\n' */
#endif
	    group = glmAddGroup(model, buf);
	    break;
	case 'f':               /* face */
	    v = n = t = 0;
	    remi = fscanf(file, "%s", buf);
	    /* can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d */
                if (strstr(buf, "//")) {
                    /* v//n */
                    sscanf(buf, "%d//%d", &v, &n);
                    remi = fscanf(file, "%d//%d", &v, &n);
                    remi = fscanf(file, "%d//%d", &v, &n);
                    numtriangles++;
                    group->numtriangles++;
                    while(fscanf(file, "%d//%d", &v, &n) > 0) {
                        numtriangles++;
                        group->numtriangles++;
                    }
                } else if (sscanf(buf, "%d/%d/%d", &v, &t, &n) == 3) {
                    /* v/t/n */
                    remi = fscanf(file, "%d/%d/%d", &v, &t, &n);
                    remi = fscanf(file, "%d/%d/%d", &v, &t, &n);
                    numtriangles++;
                    group->numtriangles++;
                    while(fscanf(file, "%d/%d/%d", &v, &t, &n) > 0) {
                        numtriangles++;
                        group->numtriangles++;
                    }
                } else if (sscanf(buf, "%d/%d", &v, &t) == 2) {
                    /* v/t */
                    remi = fscanf(file, "%d/%d", &v, &t);
                    remi = fscanf(file, "%d/%d", &v, &t);
                    numtriangles++;
                    group->numtriangles++;
                    while(fscanf(file, "%d/%d", &v, &t) > 0) {
                        numtriangles++;
                        group->numtriangles++;
                    }
                } else {
                    /* v */
                    remi = fscanf(file, "%d", &v);
                    remi = fscanf(file, "%d", &v);
                    numtriangles++;
                    group->numtriangles++;
                    while(fscanf(file, "%d", &v) > 0) {
                        numtriangles++;
                        group->numtriangles++;
                    }
                }
                break;
                
	default:
	    /* eat up rest of line */
	    rem = fgets(buf, sizeof(buf), file);
	    break;
        }
    }
  
    /* set the stats in the model structure */
    model->numvertices  = numvertices;
    model->numnormals   = numnormals;
    model->numtexcoords = numtexcoords;
    model->numtriangles = numtriangles;
  
    /* allocate memory for the triangles in each group */
    group = model->groups;
    while(group) {
	group->triangles = (GLuint*)malloc(sizeof(GLuint) * group->numtriangles);
	group->numtriangles = 0;
	group = group->next;
    }
}

/* glmSecondPass: second pass at a Wavefront OBJ file that gets all
 * the data.
 *
 * model - properly initialized GLMmodel structure
 * file  - (fopen'd) file descriptor 
 */
static GLvoid
glmSecondPass(GLMmodel* model, FILE* file) 
{
    GLuint  numvertices;        /* number of vertices in model */
    GLuint  numnormals;         /* number of normals in model */
    GLuint  numtexcoords;       /* number of texcoords in model */
    GLuint  numtriangles;       /* number of triangles in model */
    GLfloat*    vertices;           /* array of vertices  */
    GLfloat*    normals;            /* array of normals */
    GLfloat*    texcoords;          /* array of texture coordinates */
    GLMgroup* group;            /* current group pointer */
    GLuint  material;           /* current material */
    unsigned int v, n, t;
    char        buf[128];
    char *rem;
    int remi;

    /* set the pointer shortcuts */
    vertices       = model->vertices;
    normals    = model->normals;
    texcoords    = model->texcoords;
    group      = model->groups;
    
    /* on the second pass through the file, read all the data into the
    allocated arrays */
    numvertices = numnormals = numtexcoords = 1;
    numtriangles = 0;
    material = 0;
    while(fscanf(file, "%s", buf) != EOF) {
        switch(buf[0]) {
        case '#':               /* comment */
            /* eat up rest of line */
            rem = fgets(buf, sizeof(buf), file);
            break;
        case 'v':               /* v, vn, vt */
            switch(buf[1]) {
            case '\0':          /* vertex */
                remi = fscanf(file, "%f %f %f", 
                    &vertices[3 * numvertices + 0], 
                    &vertices[3 * numvertices + 1], 
                    &vertices[3 * numvertices + 2]);
                numvertices++;
                break;
            case 'n':           /* normal */
                remi = fscanf(file, "%f %f %f", 
                    &normals[3 * numnormals + 0],
                    &normals[3 * numnormals + 1], 
                    &normals[3 * numnormals + 2]);
                numnormals++;
                break;
            case 't':           /* texcoord */
                remi = fscanf(file, "%f %f", 
                    &texcoords[2 * numtexcoords + 0],
                    &texcoords[2 * numtexcoords + 1]);
                numtexcoords++;
                break;
            }
            break;
            case 'u':
                rem = fgets(buf, sizeof(buf), file);
                sscanf(buf, "%s %s", buf, buf);
                material = glmFindMaterial(model, buf);
#ifdef MATERIAL_BY_FACE
                if(!group->material && group->numtriangles)
                    group->material = material;
#else
                group->material = material;
#endif
                break;
            case 'g':               /* group */
                /* eat up rest of line */
                rem = fgets(buf, sizeof(buf), file);
#if SINGLE_STRING_GROUP_NAMES
                sscanf(buf, "%s", buf);
#else
                buf[strlen(buf)-1] = '\0';  /* nuke '\n' */
#endif
                group = glmFindGroup(model, buf);
#ifndef MATERIAL_BY_FACE
                group->material = material;
#endif
                break;
            case 'f':               /* face */
                v = n = t = 0;
		T(numtriangles).findex = -1;
#ifdef MATERIAL_BY_FACE
                if(group->material == 0)
                    group->material = material;
                T(numtriangles).material = material;
#endif
                remi = fscanf(file, "%s", buf);
                /* can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d */
                if (strstr(buf, "//")) {
                    /* v//n */
                    sscanf(buf, "%u//%u", &v, &n);
                    T(numtriangles).vindices[0] = v;
                    T(numtriangles).tindices[0] = -1;
                    T(numtriangles).nindices[0] = n;
                    remi = fscanf(file, "%u//%u", &v, &n);
                    T(numtriangles).vindices[1] = v;
                    T(numtriangles).tindices[1] = -1;
                    T(numtriangles).nindices[1] = n;
                    remi = fscanf(file, "%u//%u", &v, &n);
                    T(numtriangles).vindices[2] = v;
		    T(numtriangles).tindices[2] = -1;
                    T(numtriangles).nindices[2] = n;
                    group->triangles[group->numtriangles++] = numtriangles;
                    numtriangles++;
                    while(fscanf(file, "%u//%u", &v, &n) > 0) {
#ifdef MATERIAL_BY_FACE
                        T(numtriangles).material = material;
#endif
                        T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
                        T(numtriangles).tindices[0] = T(numtriangles-1).tindices[0];
                        T(numtriangles).nindices[0] = T(numtriangles-1).nindices[0];
                        T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
                        T(numtriangles).tindices[1] = T(numtriangles-1).tindices[2];
                        T(numtriangles).nindices[1] = T(numtriangles-1).nindices[2];
                        T(numtriangles).vindices[2] = v;
                        T(numtriangles).tindices[2] = -1;
                        T(numtriangles).nindices[2] = n;
                        group->triangles[group->numtriangles++] = numtriangles;
                        numtriangles++;
                    }
                } else if (sscanf(buf, "%u/%u/%u", &v, &t, &n) == 3) {
                    /* v/t/n */
                    T(numtriangles).vindices[0] = v;
                    T(numtriangles).tindices[0] = t;
                    T(numtriangles).nindices[0] = n;
                    remi = fscanf(file, "%u/%u/%u", &v, &t, &n);
                    T(numtriangles).vindices[1] = v;
                    T(numtriangles).tindices[1] = t;
                    T(numtriangles).nindices[1] = n;
                    remi = fscanf(file, "%u/%u/%u", &v, &t, &n);
                    T(numtriangles).vindices[2] = v;
                    T(numtriangles).tindices[2] = t;
                    T(numtriangles).nindices[2] = n;
                    group->triangles[group->numtriangles++] = numtriangles;
                    numtriangles++;
                    while(fscanf(file, "%u/%u/%u", &v, &t, &n) > 0) {
#ifdef MATERIAL_BY_FACE
                        T(numtriangles).material = material;
#endif
                        T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
                        T(numtriangles).tindices[0] = T(numtriangles-1).tindices[0];
                        T(numtriangles).nindices[0] = T(numtriangles-1).nindices[0];
                        T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
                        T(numtriangles).tindices[1] = T(numtriangles-1).tindices[2];
                        T(numtriangles).nindices[1] = T(numtriangles-1).nindices[2];
                        T(numtriangles).vindices[2] = v;
                        T(numtriangles).tindices[2] = t;
                        T(numtriangles).nindices[2] = n;
                        group->triangles[group->numtriangles++] = numtriangles;
                        numtriangles++;
                    }
                } else if (sscanf(buf, "%u/%u", &v, &t) == 2) {
                    /* v/t */
                    T(numtriangles).vindices[0] = v;
                    T(numtriangles).tindices[0] = t;
		    T(numtriangles).nindices[0] = -1;
		    remi = fscanf(file, "%u/%u", &v, &t);
                    T(numtriangles).vindices[1] = v;
                    T(numtriangles).tindices[1] = t;
		    T(numtriangles).nindices[1] = -1;
                    remi = fscanf(file, "%u/%u", &v, &t);
                    T(numtriangles).vindices[2] = v;
                    T(numtriangles).tindices[2] = t;
		    T(numtriangles).nindices[2] = -1;
                    group->triangles[group->numtriangles++] = numtriangles;
                    numtriangles++;
                    while(fscanf(file, "%u/%u", &v, &t) > 0) {
#ifdef MATERIAL_BY_FACE
                        T(numtriangles).material = material;
#endif
                        T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
                        T(numtriangles).tindices[0] = T(numtriangles-1).tindices[0];
                        T(numtriangles).nindices[0] = T(numtriangles-1).nindices[0];
                        T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
                        T(numtriangles).tindices[1] = T(numtriangles-1).tindices[2];
                        T(numtriangles).nindices[1] = T(numtriangles-1).nindices[2];
                        T(numtriangles).vindices[2] = v;
                        T(numtriangles).tindices[2] = t;
                        T(numtriangles).nindices[2] = -1;
                        group->triangles[group->numtriangles++] = numtriangles;
                        numtriangles++;
                    }
                } else {
                    /* v */
                    sscanf(buf, "%u", &v);
                    T(numtriangles).vindices[0] = v;
                    T(numtriangles).tindices[0] = -1;
		    T(numtriangles).nindices[0] = -1;
		    remi = fscanf(file, "%u", &v);
                    T(numtriangles).vindices[1] = v;
                    T(numtriangles).tindices[1] = -1;
		    T(numtriangles).nindices[1] = -1;
                    remi = fscanf(file, "%u", &v);
                    T(numtriangles).vindices[2] = v;
                    T(numtriangles).tindices[2] = -1;
		    T(numtriangles).nindices[2] = -1;
                    group->triangles[group->numtriangles++] = numtriangles;
                    numtriangles++;
                    while(fscanf(file, "%u", &v) > 0) {
#ifdef MATERIAL_BY_FACE
                        T(numtriangles).material = material;
#endif
                        T(numtriangles).vindices[0] = T(numtriangles-1).vindices[0];
                        T(numtriangles).tindices[0] = T(numtriangles-1).tindices[0];
                        T(numtriangles).nindices[0] = T(numtriangles-1).nindices[0];
                        T(numtriangles).vindices[1] = T(numtriangles-1).vindices[2];
                        T(numtriangles).tindices[1] = T(numtriangles-1).tindices[2];
                        T(numtriangles).nindices[1] = T(numtriangles-1).nindices[2];
                        T(numtriangles).vindices[2] = v;
                        T(numtriangles).tindices[2] = -1;
                        T(numtriangles).nindices[2] = -1;
                        group->triangles[group->numtriangles++] = numtriangles;
                        numtriangles++;
                    }
                }
                break;
                
            default:
                /* eat up rest of line */
                rem = fgets(buf, sizeof(buf), file);
                break;
    }
  }
  
#if 0
  /* announce the memory requirements */
  __glmWarning(" Memory: %d bytes",
      numvertices  * 3*sizeof(GLfloat) +
      numnormals   * 3*sizeof(GLfloat) * (numnormals ? 1 : 0) +
      numtexcoords * 3*sizeof(GLfloat) * (numtexcoords ? 1 : 0) +
      numtriangles * sizeof(GLMtriangle));
#endif
}


/* public functions */


/* glmUnitize: "unitize" a model by translating it to the origin and
 * scaling it to fit in a unit cube around the origin.   Returns the
 * scalefactor used.
 *
 * model - properly initialized GLMmodel structure 
 */
GLfloat
glmUnitize(GLMmodel* model)
{
    GLuint  i;
    GLfloat maxx, minx, maxy, miny, maxz, minz;
    GLfloat cx, cy, cz, w, h, d;
    GLfloat scale;
    
    assert(model);
    assert(model->vertices);
    
    /* get the max/mins */
    maxx = minx = model->vertices[3 + 0];
    maxy = miny = model->vertices[3 + 1];
    maxz = minz = model->vertices[3 + 2];
    for (i = 1; i <= model->numvertices; i++) {
        if (maxx < model->vertices[3 * i + 0])
            maxx = model->vertices[3 * i + 0];
        if (minx > model->vertices[3 * i + 0])
            minx = model->vertices[3 * i + 0];
        
        if (maxy < model->vertices[3 * i + 1])
            maxy = model->vertices[3 * i + 1];
        if (miny > model->vertices[3 * i + 1])
            miny = model->vertices[3 * i + 1];
        
        if (maxz < model->vertices[3 * i + 2])
            maxz = model->vertices[3 * i + 2];
        if (minz > model->vertices[3 * i + 2])
            minz = model->vertices[3 * i + 2];
    }
    
    /* calculate model width, height, and depth */
    w = glmAbs(maxx) + glmAbs(minx);
    h = glmAbs(maxy) + glmAbs(miny);
    d = glmAbs(maxz) + glmAbs(minz);
    
    /* calculate center of the model */
    cx = (maxx + minx) / 2.0;
    cy = (maxy + miny) / 2.0;
    cz = (maxz + minz) / 2.0;
    
    /* calculate unitizing scale factor */
    scale = 2.0 / glmMax(glmMax(w, h), d);
    
    /* translate around center then scale */
    for (i = 1; i <= model->numvertices; i++) {
        model->vertices[3 * i + 0] -= cx;
        model->vertices[3 * i + 1] -= cy;
        model->vertices[3 * i + 2] -= cz;
        model->vertices[3 * i + 0] *= scale;
        model->vertices[3 * i + 1] *= scale;
        model->vertices[3 * i + 2] *= scale;
    }
    
    return scale;
}

/* glmDimensions: Calculates the dimensions (width, height, depth) of
 * a model.
 *
 * model   - initialized GLMmodel structure
 * dimensions - array of 3 GLfloats (GLfloat dimensions[3])
 */
GLvoid
glmDimensions(GLMmodel* model, GLfloat* dimensions)
{
    GLuint i;
    GLfloat maxx, minx, maxy, miny, maxz, minz;
    
    assert(model);
    assert(model->vertices);
    assert(dimensions);
    
    /* get the max/mins */
    maxx = minx = model->vertices[3 + 0];
    maxy = miny = model->vertices[3 + 1];
    maxz = minz = model->vertices[3 + 2];
    for (i = 1; i <= model->numvertices; i++) {
        if (maxx < model->vertices[3 * i + 0])
            maxx = model->vertices[3 * i + 0];
        if (minx > model->vertices[3 * i + 0])
            minx = model->vertices[3 * i + 0];
        
        if (maxy < model->vertices[3 * i + 1])
            maxy = model->vertices[3 * i + 1];
        if (miny > model->vertices[3 * i + 1])
            miny = model->vertices[3 * i + 1];
        
        if (maxz < model->vertices[3 * i + 2])
            maxz = model->vertices[3 * i + 2];
        if (minz > model->vertices[3 * i + 2])
            minz = model->vertices[3 * i + 2];
    }
    
    /* calculate model width, height, and depth */
    dimensions[0] = glmAbs(maxx) + glmAbs(minx);
    dimensions[1] = glmAbs(maxy) + glmAbs(miny);
    dimensions[2] = glmAbs(maxz) + glmAbs(minz);
}

/* glmScale: Scales a model by a given amount.
 * 
 * model - properly initialized GLMmodel structure
 * scale - scalefactor (0.5 = half as large, 2.0 = twice as large)
 */
GLvoid
glmScale(GLMmodel* model, GLfloat scale)
{
    GLuint i;
    
    for (i = 1; i <= model->numvertices; i++) {
        model->vertices[3 * i + 0] *= scale;
        model->vertices[3 * i + 1] *= scale;
        model->vertices[3 * i + 2] *= scale;
    }
}

/* glmReverseWinding: Reverse the polygon winding for all polygons in
 * this model.   Default winding is counter-clockwise.  Also changes
 * the direction of the normals.
 * 
 * model - properly initialized GLMmodel structure 
 */
GLvoid
glmReverseWinding(GLMmodel* model)
{
    GLuint i, swap;
    
    assert(model);
    
    for (i = 0; i < model->numtriangles; i++) {
        swap = T(i).vindices[0];
        T(i).vindices[0] = T(i).vindices[2];
        T(i).vindices[2] = swap;
        
        if (model->numnormals) {
            swap = T(i).nindices[0];
            T(i).nindices[0] = T(i).nindices[2];
            T(i).nindices[2] = swap;
        }
        
        if (model->numtexcoords) {
            swap = T(i).tindices[0];
            T(i).tindices[0] = T(i).tindices[2];
            T(i).tindices[2] = swap;
        }
    }
    
    /* reverse facet normals */
    for (i = 1; i <= model->numfacetnorms; i++) {
        model->facetnorms[3 * i + 0] = -model->facetnorms[3 * i + 0];
        model->facetnorms[3 * i + 1] = -model->facetnorms[3 * i + 1];
        model->facetnorms[3 * i + 2] = -model->facetnorms[3 * i + 2];
    }
    
    /* reverse vertex normals */
    for (i = 1; i <= model->numnormals; i++) {
        model->normals[3 * i + 0] = -model->normals[3 * i + 0];
        model->normals[3 * i + 1] = -model->normals[3 * i + 1];
        model->normals[3 * i + 2] = -model->normals[3 * i + 2];
    }
}

/* glmFacetNormals: Generates facet normals for a model (by taking the
 * cross product of the two vectors derived from the sides of each
 * triangle).  Assumes a counter-clockwise winding.
 *
 * model - initialized GLMmodel structure
 */
GLvoid
glmFacetNormals(GLMmodel* model)
{
    GLuint  i;
    GLfloat u[3];
    GLfloat v[3];
    
    assert(model);
    assert(model->vertices);
    
    /* clobber any old facetnormals */
    if (model->facetnorms) {
	free(model->facetnorms);
    }
    /* allocate memory for the new facet normals */
    model->numfacetnorms = model->numtriangles;
    model->facetnorms = (GLfloat*)malloc(sizeof(GLfloat) *
					 3 * (model->numfacetnorms + 1));

    for (i = 0; i < model->numtriangles; i++) {
	T(i).findex = i+1;
        
	u[0] = model->vertices[3 * T(i).vindices[1] + 0] -
	    model->vertices[3 * T(i).vindices[0] + 0];
	u[1] = model->vertices[3 * T(i).vindices[1] + 1] -
	    model->vertices[3 * T(i).vindices[0] + 1];
	u[2] = model->vertices[3 * T(i).vindices[1] + 2] -
	    model->vertices[3 * T(i).vindices[0] + 2];
	    
	v[0] = model->vertices[3 * T(i).vindices[2] + 0] -
	    model->vertices[3 * T(i).vindices[0] + 0];
	v[1] = model->vertices[3 * T(i).vindices[2] + 1] -
	    model->vertices[3 * T(i).vindices[0] + 1];
	v[2] = model->vertices[3 * T(i).vindices[2] + 2] -
	    model->vertices[3 * T(i).vindices[0] + 2];
        
	glmCross(u, v, &model->facetnorms[3 * (i+1)]);
	glmNormalize(&model->facetnorms[3 * (i+1)]);
    }

}

/* glmVertexNormals: Generates smooth vertex normals for a model.
 * First builds a list of all the triangles each vertex is in.   Then
 * loops through each vertex in the the list averaging all the facet
 * normals of the triangles each vertex is in.   Finally, sets the
 * normal index in the triangle for the vertex to the generated smooth
 * normal.   If the dot product of a facet normal and the facet normal
 * associated with the first triangle in the list of triangles the
 * current vertex is in is greater than the cosine of the angle
 * parameter to the function, that facet normal is not added into the
 * average normal calculation and the corresponding vertex is given
 * the facet normal.  This tends to preserve hard edges.  The angle to
 * use depends on the model, but 90 degrees is usually a good start.
 *
 * model - initialized GLMmodel structure
 * angle - maximum angle (in degrees) to smooth across
 */
GLvoid
glmVertexNormals(GLMmodel* model, GLfloat angle, GLboolean keep_existing)
{
    GLMnode*    node;
    GLMnode*    tail;
    GLMnode** members;
    GLfloat*    normals;
    GLuint  numnormals;
    GLfloat average[3];
    GLfloat dot, cos_angle;
    GLuint  i;
    
    DBG_(__glmWarning( "glmVertexNormals(): begin"));
    assert(model);
    assert(model->facetnorms);
    
    /* calculate the cosine of the angle (in degrees) */
    cos_angle = cos(angle * M_PI / 180.0);

    if(keep_existing) {
	numnormals = model->numnormals + 1; /* index of the next normal */
    }
    else {
	/* nuke any previous normals */
	if (model->normals) {
	    free(model->normals);
	}
	/* allocate space for new normals */
	model->numnormals = model->numtriangles * 3; /* 3 normals per triangle */
	model->normals = (GLfloat*)malloc(sizeof(GLfloat)* 3* (model->numnormals + 1));
	numnormals = 1;
    }

    /* allocate a structure that will hold a linked list of triangle
    indices for each vertex */
    members = (GLMnode**)malloc(sizeof(GLMnode*) * (model->numvertices + 1));
    for (i = 1; i <= model->numvertices; i++)
        members[i] = NULL;
    
    /* for every triangle, create a node for each vertex in it */
    for (i = 0; i < model->numtriangles; i++) {
	assert(T(i).vindices[0] <= model->numvertices);
	assert(T(i).vindices[1] <= model->numvertices);
	assert(T(i).vindices[2] <= model->numvertices);

        node = (GLMnode*)malloc(sizeof(GLMnode));
        node->index = i;
        node->next  = members[T(i).vindices[0]];
        members[T(i).vindices[0]] = node;
        
        node = (GLMnode*)malloc(sizeof(GLMnode));
        node->index = i;
        node->next  = members[T(i).vindices[1]];
        members[T(i).vindices[1]] = node;
        
        node = (GLMnode*)malloc(sizeof(GLMnode));
        node->index = i;
        node->next  = members[T(i).vindices[2]];
        members[T(i).vindices[2]] = node;
    }
    
    /* calculate the average normal for each vertex */
    for (i = 1; i <= model->numvertices; i++) {
        int avg_index;

	/* calculate an average normal for this vertex by averaging the
	   facet normal of every triangle this vertex is in */
        node = members[i];
        if (!node)
            __glmWarning( "glmVertexNormals(): vertex %d w/o a triangle", i);
        average[0] = 0.0; average[1] = 0.0; average[2] = 0.0;
        while (node) {
	    node->averaged = GL_FALSE;
	    if ((T(node->index).findex != -1) && (T(members[i]->index).findex != -1)) {
		/* only average if the dot product of the angle between the two
		   facet normals is greater than the cosine of the threshold
		   angle -- or, said another way, the angle between the two
		   facet normals is less than (or equal to) the threshold angle */
		assert(T(node->index).findex <= model->numfacetnorms);
		assert(T(members[i]->index).findex <= model->numfacetnorms);
		dot = glmDot(&model->facetnorms[3 * T(node->index).findex],
			     &model->facetnorms[3 * T(members[i]->index).findex]);
		if (dot > cos_angle) {
		    node->averaged = GL_TRUE;
		    average[0] += model->facetnorms[3 * T(node->index).findex + 0];
		    average[1] += model->facetnorms[3 * T(node->index).findex + 1];
		    average[2] += model->facetnorms[3 * T(node->index).findex + 2];
		}
	    }
            node = node->next;
        }
        
        /* set the normal of this vertex in each triangle it is in */
	avg_index = -1;
        node = members[i];
        while (node) {
	    int j;

            if (node->averaged) {
                /* if this node was averaged, use the average normal */
		for (j = 0; j<3; j++) {
		    assert(T(node->index).vindices[j] <= model->numvertices);
		    if (T(node->index).vindices[j] == i) {
			if(T(node->index).nindices[j] > numnormals);
			assert(T(node->index).nindices[j] == -1 || T(node->index).nindices[j] <= model->numnormals);
			if (!keep_existing || T(node->index).nindices[j] == -1) {
			    if (avg_index == -1) {
				while (model->numnormals < numnormals) {
				    DBG_(__glmWarning( "glmVertexNormals(): realloc %d+100\n", model->numnormals+100));
				    /* allocate 1000 more normals */
				    model->numnormals += 1000;
				    model->normals = (GLfloat*)realloc(model->normals, sizeof(GLfloat)* 3 * (model->numnormals+1));
				}
				
				/* normalize the averaged normal */
				glmNormalize(average);
				
				/* add the normal to the vertex normals list */
				assert(model->numnormals >= numnormals);
				model->normals[3 * numnormals + 0] = average[0];
				model->normals[3 * numnormals + 1] = average[1];
				model->normals[3 * numnormals + 2] = average[2];
				avg_index = numnormals;
				numnormals++;
			    }
			    T(node->index).nindices[j] = avg_index;
			}
		    }
		}
            } else if (T(node->index).findex != -1) {
		int discard = 1;

		while (model->numnormals < numnormals) {
		    __glmWarning( "glmVertexNormals(): realloc %d+100\n", model->numnormals+100);
		    /* allocate 100 more normals */
		    model->numnormals += 100;
		    model->normals = (GLfloat*)realloc(model->normals, sizeof(GLfloat)* 3 * (model->numnormals+1));
		}
                assert(T(node->index).findex == -1 || T(node->index).findex <= model->numfacetnorms);
		assert(model->numnormals >= numnormals);
		/* if this node wasn't averaged, use the facet normal */
                model->normals[3 * numnormals + 0] = 
                    model->facetnorms[3 * T(node->index).findex + 0];
                model->normals[3 * numnormals + 1] = 
                    model->facetnorms[3 * T(node->index).findex + 1];
                model->normals[3 * numnormals + 2] = 
                    model->facetnorms[3 * T(node->index).findex + 2];
		for (j = 0; j<3; j++) {
		    assert(T(node->index).vindices[j] <= model->numvertices);
		    if (T(node->index).vindices[j] == i) {
			assert(T(node->index).nindices[j] == -1 || T(node->index).nindices[j] <= model->numnormals);
			if (!keep_existing || T(node->index).nindices[j] == -1) {
			    discard = 0;
			    T(node->index).nindices[j] = numnormals;
			}
		    }
		}
		if (!discard)
		    numnormals++;
            } else {
		for (j = 0; j<3; j++) {
		    assert(T(node->index).vindices[j] <= model->numvertices);
		    if (T(node->index).vindices[j] == i) {
			assert(T(node->index).nindices[j] <= model->numnormals);
			if (!keep_existing)
			    T(node->index).nindices[j] = -1;
		    }
		}
	    }
            node = node->next;
        }
    }
    
    model->numnormals = numnormals - 1;
    
    /* free the member information */
    for (i = 1; i <= model->numvertices; i++) {
        node = members[i];
        while (node) {
            tail = node;
            node = node->next;
            free(tail);
        }
    }
    free(members);
    
    /* pack the normals array (we previously allocated the maximum
       number of normals that could possibly be created (numtriangles *
       3), so get rid of some of them (usually alot unless none of the
       facet normals were averaged)) */
    normals = model->normals;
    model->normals = (GLfloat*)malloc(sizeof(GLfloat)* 3* (model->numnormals+1));
    for (i = 1; i <= model->numnormals; i++) {
        model->normals[3 * i + 0] = normals[3 * i + 0];
        model->normals[3 * i + 1] = normals[3 * i + 1];
        model->normals[3 * i + 2] = normals[3 * i + 2];
    }
    free(normals);
    DBG_(__glmWarning( "glmVertexNormals(): end"));
}


/* glmLinearTexture: Generates texture coordinates according to a
 * linear projection of the texture map.  It generates these by
 * linearly mapping the vertices onto a square.
 *
 * model - pointer to initialized GLMmodel structure
 */
GLvoid
glmLinearTexture(GLMmodel* model)
{
    GLMgroup *group;
    GLfloat dimensions[3];
    GLfloat x, y, scalefactor;
    GLuint i;
    
    assert(model);
    
    if (model->texcoords)
        free(model->texcoords);
    model->numtexcoords = model->numvertices;
    model->texcoords=(GLfloat*)malloc(sizeof(GLfloat)*2*(model->numtexcoords+1));
    
    glmDimensions(model, dimensions);
    scalefactor = 2.0 / 
        glmAbs(glmMax(glmMax(dimensions[0], dimensions[1]), dimensions[2]));
    
    /* do the calculations */
    for(i = 1; i <= model->numvertices; i++) {
        x = model->vertices[3 * i + 0] * scalefactor;
        y = model->vertices[3 * i + 2] * scalefactor;
        model->texcoords[2 * i + 0] = (x + 1.0) / 2.0;
        model->texcoords[2 * i + 1] = (y + 1.0) / 2.0;
    }
    
    /* go through and put texture coordinate indices in all the triangles */
    group = model->groups;
    while(group) {
        for(i = 0; i < group->numtriangles; i++) {
            T(group->triangles[i]).tindices[0] = T(group->triangles[i]).vindices[0];
            T(group->triangles[i]).tindices[1] = T(group->triangles[i]).vindices[1];
            T(group->triangles[i]).tindices[2] = T(group->triangles[i]).vindices[2];
        }    
        group = group->next;
    }
    
#if 0
    __glmWarning("glmLinearTexture(): generated %d linear texture coordinates",
		 model->numtexcoords);
#endif
}

/* glmSpheremapTexture: Generates texture coordinates according to a
 * spherical projection of the texture map.  Sometimes referred to as
 * spheremap, or reflection map texture coordinates.  It generates
 * these by using the normal to calculate where that vertex would map
 * onto a sphere.  Since it is impossible to map something flat
 * perfectly onto something spherical, there is distortion at the
 * poles.  This particular implementation causes the poles along the X
 * axis to be distorted.
 *
 * model - pointer to initialized GLMmodel structure
 */
GLvoid
glmSpheremapTexture(GLMmodel* model)
{
    GLMgroup* group;
    GLfloat theta, phi, rho, x, y, z, r;
    GLuint i;
    
    assert(model);
    assert(model->normals);
    
    if (model->texcoords)
        free(model->texcoords);
    model->numtexcoords = model->numnormals;
    model->texcoords=(GLfloat*)malloc(sizeof(GLfloat)*2*(model->numtexcoords+1));
    
    for (i = 1; i <= model->numnormals; i++) {
        z = model->normals[3 * i + 0];  /* re-arrange for pole distortion */
        y = model->normals[3 * i + 1];
        x = model->normals[3 * i + 2];
        r = sqrt((x * x) + (y * y));
        rho = sqrt((r * r) + (z * z));
        
        if(r == 0.0) {
            theta = 0.0;
            phi = 0.0;
        } else {
            if(z == 0.0)
                phi = 3.14159265 / 2.0;
            else
                phi = acos(z / rho);
            
            if(y == 0.0)
                theta = 3.141592365 / 2.0;
            else
                theta = asin(y / r) + (3.14159265 / 2.0);
        }
        
        model->texcoords[2 * i + 0] = theta / 3.14159265;
        model->texcoords[2 * i + 1] = phi / 3.14159265;
    }
    
    /* go through and put texcoord indices in all the triangles */
    group = model->groups;
    while(group) {
        for (i = 0; i < group->numtriangles; i++) {
            T(group->triangles[i]).tindices[0] = T(group->triangles[i]).nindices[0];
            T(group->triangles[i]).tindices[1] = T(group->triangles[i]).nindices[1];
            T(group->triangles[i]).tindices[2] = T(group->triangles[i]).nindices[2];
        }
        group = group->next;
    }
}

/* glmDelete: Deletes a GLMmodel structure.
 *
 * model - initialized GLMmodel structure
 */
GLvoid
glmDelete(GLMmodel* model)
{
    GLMgroup* group;
    GLuint i;
    
    assert(model);
    
    if (model->pathname)     free(model->pathname);
    if (model->mtllibname) free(model->mtllibname);
    if (model->vertices)     free(model->vertices);
    if (model->normals)  free(model->normals);
    if (model->texcoords)  free(model->texcoords);
    if (model->facetnorms) free(model->facetnorms);
    if (model->triangles)  free(model->triangles);
    if (model->materials) {
        for (i = 0; i < model->nummaterials; i++)
	    {
		free(model->materials[i].name);
	    }
        free(model->materials);
    }
    if (model->textures) {
        for (i = 0; i < model->numtextures; i++) {
            free(model->textures[i].name);
            glDeleteTextures(1,&model->textures[i].id);
        }
        free(model->textures);
    }
    while(model->groups) {
        group = model->groups;
        model->groups = model->groups->next;
        free(group->name);
        free(group->triangles);
        free(group);
    }
    
    free(model);
}

/* glmReadOBJ: Reads a model description from a Wavefront .OBJ file.
 * Returns a pointer to the created object which should be free'd with
 * glmDelete().
 *
 * filename - name of the file containing the Wavefront .OBJ format data.  
 */
GLMmodel* 
glmReadOBJ(const char* filename)
{
    GLMmodel* model;
    FILE*   file;
    int i, j;

    /* open the file */
    file = fopen(filename, "r");
    if (!file) {
        __glmFatalError( "glmReadOBJ() failed: can't open data file \"%s\".",
			 filename);
    }

    /* allocate a new model */
    model = (GLMmodel*)malloc(sizeof(GLMmodel));
    model->pathname    = __glmStrdup(filename);
    model->mtllibname    = NULL;
    model->numvertices   = 0;
    model->vertices    = NULL;
    model->numnormals    = 0;
    model->normals     = NULL;
    model->numtexcoords  = 0;
    model->texcoords       = NULL;
    model->numfacetnorms = 0;
    model->facetnorms    = NULL;
    model->numtriangles  = 0;
    model->triangles       = NULL;
    model->nummaterials  = 0;
    model->materials       = NULL;
    model->numtextures  = 0;
    model->textures       = NULL;
    model->numgroups       = 0;
    model->groups      = NULL;
    model->position[0]   = 0.0;
    model->position[1]   = 0.0;
    model->position[2]   = 0.0;
    
    /* make a first pass through the file to get a count of the number
       of vertices, normals, texcoords & triangles */
    glmFirstPass(model, file);
    
    /* allocate memory */
    model->vertices = (GLfloat*)malloc(sizeof(GLfloat) *
				       3 * (model->numvertices + 1));
    model->triangles = (GLMtriangle*)malloc(sizeof(GLMtriangle) *
					    model->numtriangles);
    if (model->numnormals) {
        model->normals = (GLfloat*)malloc(sizeof(GLfloat) *
					  3 * (model->numnormals + 1));
    }
    if (model->numtexcoords) {
        model->texcoords = (GLfloat*)malloc(sizeof(GLfloat) *
					    2 * (model->numtexcoords + 1));
    }
    
    /* rewind to beginning of file and read in the data this pass */
    rewind(file);
    
    glmSecondPass(model, file);

    /* facet normals are not in the file, we have to compute them anyway */
    glmFacetNormals(model);

    /* verify the indices */
    for (i = 0; i < model->numtriangles; i++) {
	if (T(i).findex != -1)
	    if (T(i).findex <= 0 || T(i).findex > model->numfacetnorms)
		__glmFatalError("facet index for triangle %d out of bounds (%d > %d)\n", i, T(i).findex, model->numfacetnorms);
	for (j=0; j<3; j++) {
	    if (T(i).nindices[j] != -1)
		if (T(i).nindices[j] <= 0 || T(i).nindices[j] > model->numnormals)
		    __glmFatalError("normal index for triangle %d out of bounds (%d > %d)\n", i, T(i).nindices[j], model->numnormals);
	    if (T(i).vindices[j] != -1)
		if (T(i).vindices[j] <= 0 || T(i).vindices[j] > model->numvertices)
		    __glmFatalError("vertex index for triangle %d out of bounds (%d > %d)\n", i, T(i).vindices[j], model->numvertices);
	}
    }

    /* close the file */
    fclose(file);
    
    return model;
}

/* glmWriteOBJ: Writes a model description in Wavefront .OBJ format to
 * a file.
 *
 * model - initialized GLMmodel structure
 * filename - name of the file to write the Wavefront .OBJ format data to
 * mode  - a bitwise or of values describing what is written to the file
 *             GLM_NONE     -  render with only vertices
 *             GLM_FLAT     -  render with facet normals
 *             GLM_SMOOTH   -  render with vertex normals
 *             GLM_TEXTURE  -  render with texture coords
 *             GLM_COLOR    -  render with colors (color material)
 *             GLM_MATERIAL -  render with materials
 *             GLM_COLOR and GLM_MATERIAL should not both be specified.  
 *             GLM_FLAT and GLM_SMOOTH should not both be specified.  
 */
GLvoid
glmWriteOBJ(GLMmodel* model, char* filename, GLuint mode)
{
    GLuint  i;
    FILE*   file;
    GLMgroup* group;
    GLuint material = -1;
    
    assert(model);
    
    /* do a bit of warning */
    if (mode & GLM_FLAT && !model->facetnorms) {
        __glmWarning("glmWriteOBJ() warning: flat normal output requested "
		     "with no facet normals defined.");
        mode &= ~GLM_FLAT;
    }
    if (mode & GLM_SMOOTH && !model->normals) {
        __glmWarning("glmWriteOBJ() warning: smooth normal output requested "
		     "with no normals defined.");
        mode &= ~GLM_SMOOTH;
    }
    if (mode & GLM_TEXTURE && !model->texcoords) {
        __glmWarning("glmWriteOBJ() warning: texture coordinate output requested "
		     "with no texture coordinates defined.");
        mode &= ~GLM_TEXTURE;
    }
    if (mode & GLM_FLAT && mode & GLM_SMOOTH) {
        __glmWarning("glmWriteOBJ() warning: flat normal output requested "
		     "and smooth normal output requested (using smooth).");
        mode &= ~GLM_FLAT;
    }
    if (mode & GLM_COLOR && !model->materials) {
        __glmWarning("glmWriteOBJ() warning: color output requested "
		     "with no colors (materials) defined.");
        mode &= ~GLM_COLOR;
    }
    if (mode & GLM_MATERIAL && !model->materials) {
        __glmWarning("glmWriteOBJ() warning: material output requested "
		     "with no materials defined.");
        mode &= ~GLM_MATERIAL;
    }
    if (mode & GLM_COLOR && mode & GLM_MATERIAL) {
        __glmWarning("glmWriteOBJ() warning: color and material output requested "
		     "outputting only materials.");
        mode &= ~GLM_COLOR;
    }
    
    
    /* open the file */
    file = fopen(filename, "w");
    if (!file) {
        __glmFatalError( "glmWriteOBJ() failed: can't open file \"%s\" to write.",
			 filename);
    }
    
    /* spit out a header */
    fprintf(file, "#  \n");
    fprintf(file, "#  Wavefront OBJ generated by GLM library\n");
    fprintf(file, "#  \n");
    fprintf(file, "#  GLM library\n");
    fprintf(file, "#  Nate Robins\n");
    fprintf(file, "#  ndr@pobox.com\n");
    fprintf(file, "#  http://www.pobox.com/~ndr\n");
    fprintf(file, "#  \n");
    
    if (mode & GLM_MATERIAL && model->mtllibname) {
        fprintf(file, "\nmtllib %s\n\n", model->mtllibname);
        glmWriteMTL(model, filename, model->mtllibname);
    }
    
    /* spit out the vertices */
    fprintf(file, "\n");
    fprintf(file, "# %d vertices\n", model->numvertices);
    for (i = 1; i <= model->numvertices; i++) {
        fprintf(file, "v %f %f %f\n", 
		model->vertices[3 * i + 0],
		model->vertices[3 * i + 1],
		model->vertices[3 * i + 2]);
    }
    
    /* spit out the smooth/flat normals */
    if (mode & GLM_SMOOTH) {
        fprintf(file, "\n");
        fprintf(file, "# %u normals\n", (unsigned int)model->numnormals);
        for (i = 1; i <= model->numnormals; i++) {
            fprintf(file, "vn %f %f %f\n", 
		    model->normals[3 * i + 0],
		    model->normals[3 * i + 1],
		    model->normals[3 * i + 2]);
        }
    } else if (mode & GLM_FLAT) {
        fprintf(file, "\n");
        fprintf(file, "# %d normals\n", model->numfacetnorms);
        for (i = 1; i <= model->numnormals; i++) {
            fprintf(file, "vn %f %f %f\n", 
		    model->facetnorms[3 * i + 0],
		    model->facetnorms[3 * i + 1],
		    model->facetnorms[3 * i + 2]);
        }
    }
    
    /* spit out the texture coordinates */
    if (mode & GLM_TEXTURE) {
        fprintf(file, "\n");
        fprintf(file, "# %d texcoords\n", model->numtexcoords);
        for (i = 1; i <= model->numtexcoords; i++) {
            fprintf(file, "vt %f %f\n", 
		    model->texcoords[2 * i + 0],
		    model->texcoords[2 * i + 1]);
        }
    }
    
    fprintf(file, "\n");
    fprintf(file, "# %d groups\n", model->numgroups);
    fprintf(file, "# %d faces (triangles)\n", model->numtriangles);
    fprintf(file, "\n");
    
    group = model->groups;
    while(group) {
        fprintf(file, "g %s\n", group->name);
        if (mode & GLM_MATERIAL) {
            fprintf(file, "usemtl %s\n", model->materials[group->material].name);
#ifdef MATERIAL_BY_FACE
            material = group->material;
#endif
        }
        for (i = 0; i < group->numtriangles; i++) {
#ifdef MATERIAL_BY_FACE
            if(T(group->triangles[i]).material && T(group->triangles[i]).material != material) {
                material = T(group->triangles[i]).material;
                fprintf(file, "usemtl %s\n", model->materials[material].name);
            }
#endif
            if (mode & GLM_SMOOTH && mode & GLM_TEXTURE) {
                fprintf(file, "f %d/%d/%d %d/%d/%d %d/%d/%d\n",
			T(group->triangles[i]).vindices[0], 
			T(group->triangles[i]).nindices[0], 
			T(group->triangles[i]).tindices[0],
			T(group->triangles[i]).vindices[1],
			T(group->triangles[i]).nindices[1],
			T(group->triangles[i]).tindices[1],
			T(group->triangles[i]).vindices[2],
			T(group->triangles[i]).nindices[2],
			T(group->triangles[i]).tindices[2]);
            } else if (mode & GLM_FLAT && mode & GLM_TEXTURE) {
                fprintf(file, "f %d/%d %d/%d %d/%d\n",
			T(group->triangles[i]).vindices[0],
			T(group->triangles[i]).findex,
			T(group->triangles[i]).vindices[1],
			T(group->triangles[i]).findex,
			T(group->triangles[i]).vindices[2],
			T(group->triangles[i]).findex);
            } else if (mode & GLM_TEXTURE) {
                fprintf(file, "f %d/%d %d/%d %d/%d\n",
			T(group->triangles[i]).vindices[0],
			T(group->triangles[i]).tindices[0],
			T(group->triangles[i]).vindices[1],
			T(group->triangles[i]).tindices[1],
			T(group->triangles[i]).vindices[2],
			T(group->triangles[i]).tindices[2]);
            } else if (mode & GLM_SMOOTH) {
                fprintf(file, "f %d//%d %d//%d %d//%d\n",
			T(group->triangles[i]).vindices[0],
			T(group->triangles[i]).nindices[0],
			T(group->triangles[i]).vindices[1],
			T(group->triangles[i]).nindices[1],
			T(group->triangles[i]).vindices[2], 
			T(group->triangles[i]).nindices[2]);
            } else if (mode & GLM_FLAT) {
                fprintf(file, "f %d//%d %d//%d %d//%d\n",
			T(group->triangles[i]).vindices[0], 
			T(group->triangles[i]).findex,
			T(group->triangles[i]).vindices[1],
			T(group->triangles[i]).findex,
			T(group->triangles[i]).vindices[2],
			T(group->triangles[i]).findex);
            } else {
                fprintf(file, "f %d %d %d\n",
			T(group->triangles[i]).vindices[0],
			T(group->triangles[i]).vindices[1],
			T(group->triangles[i]).vindices[2]);
            }
        }
        fprintf(file, "\n");
        group = group->next;
    }
    
    fclose(file);
}

/* glmDraw: Renders the model to the current OpenGL context using the
 * mode specified.
 *
 * model - initialized GLMmodel structure
 * mode  - a bitwise OR of values describing what is to be rendered.
 *             GLM_NONE     -  render with only vertices
 *             GLM_FLAT     -  render with facet normals
 *             GLM_SMOOTH   -  render with vertex normals
 *             GLM_TEXTURE  -  render with texture coords
 *             GLM_COLOR    -  render with colors (color material)
 *             GLM_MATERIAL -  render with materials
 *             GLM_COLOR and GLM_MATERIAL should not both be specified.  
 *             GLM_FLAT and GLM_SMOOTH should not both be specified.  
 */
GLvoid
glmDraw(GLMmodel* model, GLuint mode)
{
    GLuint i, j;
    GLuint blenditer, newmaterial, newtexture;
    GLuint blendmodel = 0;
    GLMgroup* group;
    GLMtriangle* triangle;
    GLuint material, map_diffuse;
    GLMmaterial* materialp;

    assert(model);
    assert(model->vertices);
    
    /* do a bit of warning */
    if (mode & GLM_FLAT && !model->facetnorms) {
        __glmWarning("glmDraw() warning: flat render mode requested "
		     "with no facet normals defined.");
        mode &= ~GLM_FLAT;
    }
    if (mode & GLM_SMOOTH && !model->normals) {
        __glmWarning("glmDraw() warning: smooth render mode requested "
		     "with no normals defined.");
        mode &= ~GLM_SMOOTH;
    }
    if (mode & GLM_TEXTURE && !model->texcoords) {
        // Warnings ignored for DRC project
        //__glmWarning("glmDraw() warning: texture render mode requested "
	//	     "with no texture coordinates defined.");
        mode &= ~GLM_TEXTURE;
    }
    if (mode & GLM_FLAT && mode & GLM_SMOOTH) {
        __glmWarning("glmDraw() warning: flat render mode requested "
		     "and smooth render mode requested (using smooth).");
        mode &= ~GLM_FLAT;
    }
    if (mode & GLM_COLOR && !model->materials) {
        __glmWarning("glmDraw() warning: color render mode requested "
		     "with no materials defined.");
        mode &= ~GLM_COLOR;
    }
    if (mode & GLM_MATERIAL && !model->materials) {
        // Warnings ignored for DRC project
        //__glmWarning("glmDraw() warning: material render mode requested "
	//	     "with no materials defined.");
        mode &= ~GLM_MATERIAL;
    }
    if (mode & GLM_COLOR && mode & GLM_MATERIAL) {
        __glmWarning("glmDraw() warning: color and material render mode requested "
		     "using only material mode.");
        mode &= ~GLM_COLOR;
    }
    if (mode & GLM_COLOR)
        glEnable(GL_COLOR_MATERIAL);
    else if (mode & GLM_MATERIAL)
        glDisable(GL_COLOR_MATERIAL);
    if (mode & GLM_TEXTURE) {
        glEnable(_glmTextureTarget);
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    }
#ifdef GLM_2_SIDED
    if(mode & GLM_2_SIDED)
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    else
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
#endif

    /* perhaps this loop should be unrolled into material, color, flat,
       smooth, etc. loops?  since most cpu's have good branch prediction
       schemes (and these branches will always go one way), probably
       wouldn't gain too much?  */
    
    /* CHEESY BLENDING (AKA: NO SORTING)
       If model has blending, use two pass approach, alpha items last */
    for(blenditer = 0; blenditer<2; blenditer++) {
	int blending = 0;
	newmaterial = 0;
	material = -1;
	materialp = NULL;
	newtexture = 0;
	map_diffuse = -1;	/* default material */
	group = model->groups;
	while (group) {
	    if (mode & (GLM_MATERIAL|GLM_COLOR|GLM_TEXTURE)) {
		material = group->material;
		materialp = &model->materials[material];
		blending = (materialp->diffuse[3] < 1.0);
		blendmodel |= blending;
		newmaterial = 1;
		if(materialp->map_diffuse != map_diffuse) {
		    newtexture = 1;
		    map_diffuse = materialp->map_diffuse;
		}
	    }
        
	    glBegin(GL_TRIANGLES);
	    for (i = 0; i < group->numtriangles; i++) {
		triangle = &T(group->triangles[i]);

#ifdef MATERIAL_BY_FACE
		if (mode & (GLM_MATERIAL|GLM_COLOR|GLM_TEXTURE)) {
		    /* if the triangle has a different material than the last drawn triangle */
		    if(triangle->material && triangle->material != material) {
			material = triangle->material;
			materialp = &model->materials[material];
			blending = (materialp->diffuse[3] < 1.0);
			blendmodel |= blending;
			newmaterial = 1;
			if(materialp->map_diffuse != map_diffuse) {
			    newtexture = 1;
			    map_diffuse = materialp->map_diffuse;
			}
		    }
		}
#endif

		/* render only if in the right blending pass */
		if(blending == blenditer) {
		    if(newmaterial) {
			newmaterial = 0;
			if (mode & GLM_TEXTURE) {
			    if(newtexture) {
				newtexture = 0;
				glEnd();
				if(map_diffuse == -1)
				    glBindTexture(_glmTextureTarget, 0);
				else
				    glBindTexture(_glmTextureTarget, model->textures[map_diffuse].id);
				glBegin(GL_TRIANGLES);
			    }
			}
			if (mode & GLM_MATERIAL) {
			    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, materialp->ambient);
			    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, materialp->diffuse);
			    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, materialp->specular);
			    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, materialp->shininess);
			}        
			if (mode & GLM_COLOR) {
			    glColor3fv(materialp->diffuse);
			}
		    }
		
		    if (mode & GLM_FLAT)
                glNormal3fv(&model->facetnorms[3 * triangle->findex]);
		
		    for (j=0; j<3; j++) {
                if (mode & GLM_SMOOTH && (triangle->nindices[j]!=-1)) {
                    assert(triangle->nindices[j]>=1 && triangle->nindices[j]<=model->numnormals);
                    glNormal3fv(&model->normals[3 * triangle->nindices[j]]);
                }
                if (mode & GLM_TEXTURE && (triangle->tindices[j]!=-1) && map_diffuse != -1) {
                    assert(map_diffuse >= 0 && map_diffuse < model->numtextures);
                    assert(triangle->tindices[j]>=1 && triangle->tindices[j]<=model->numtexcoords);
                    glTexCoord2f(model->texcoords[2 * triangle->tindices[j]]*model->textures[map_diffuse].width,model->texcoords[2 * triangle->tindices[j] + 1]*model->textures[map_diffuse].height);
                }
                assert(triangle->vindices[j]>=1 && triangle->vindices[j]<=model->numvertices);
                glVertex3fv(&model->vertices[3 * triangle->vindices[j]]);
		    }
		}
        
	    }
	    glEnd();
        
	    group = group->next;
	}
	if(!blendmodel)
	    break;			/* jump out of the for(blenditer) */
	assert(blendmodel);
	/* Prep for second pass with alpha items */
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE); /* Type Of Blending To Perform */
	glDepthMask(GL_FALSE);	/* Turn off depth mask */
    } /* for(blenditer) */
    if(blendmodel) {
	glDepthMask(GL_TRUE);	/* DISABLE Blending conditions */
	glDisable(GL_BLEND);
    }
}

/* glmList: Generates and returns a display list for the model using
 * the mode specified.
 *
 * model - initialized GLMmodel structure
 * mode  - a bitwise OR of values describing what is to be rendered.
 *             GLM_NONE     -  render with only vertices
 *             GLM_FLAT     -  render with facet normals
 *             GLM_SMOOTH   -  render with vertex normals
 *             GLM_TEXTURE  -  render with texture coords
 *             GLM_COLOR    -  render with colors (color material)
 *             GLM_MATERIAL -  render with materials
 *             GLM_COLOR and GLM_MATERIAL should not both be specified.  
 * GLM_FLAT and GLM_SMOOTH should not both be specified.  
 */
GLuint
glmList(GLMmodel* model, GLuint mode)
{
    GLuint list;
    
    list = glGenLists(1);
    glNewList(list, GL_COMPILE);
    glmDraw(model, mode);
    glEndList();
    
    return list;
}

/* glmWeld: eliminate (weld) vectors that are within an epsilon of
 * each other.
 *
 * model   - initialized GLMmodel structure
 * epsilon     - maximum difference between vertices
 *               ( 0.00001 is a good start for a unitized model)
 *
 */
GLvoid
glmWeld(GLMmodel* model, GLfloat epsilon)
{
    GLfloat* vectors;
    GLfloat* copies;
    GLuint   numvectors;
    GLuint   i;
    
    /* vertices */
    numvectors = model->numvertices;
    vectors  = model->vertices;
    copies = glmWeldVectors(vectors, &numvectors, epsilon);
    
#if 0
    __glmWarning("glmWeld(): %d redundant vertices.", 
		 model->numvertices - numvectors - 1);
#endif
    
    for (i = 0; i < model->numtriangles; i++) {
        T(i).vindices[0] = (GLuint)vectors[3 * T(i).vindices[0] + 0];
        T(i).vindices[1] = (GLuint)vectors[3 * T(i).vindices[1] + 0];
        T(i).vindices[2] = (GLuint)vectors[3 * T(i).vindices[2] + 0];
    }
    
    /* free space for old vertices */
    free(vectors);
    
    /* allocate space for the new vertices */
    model->numvertices = numvectors;
    model->vertices = (GLfloat*)malloc(sizeof(GLfloat) * 
				       3 * (model->numvertices + 1));
    
    /* copy the optimized vertices into the actual vertex list */
    for (i = 1; i <= model->numvertices; i++) {
        model->vertices[3 * i + 0] = copies[3 * i + 0];
        model->vertices[3 * i + 1] = copies[3 * i + 1];
        model->vertices[3 * i + 2] = copies[3 * i + 2];
    }
    
    free(copies);
}

#ifdef AVL
//AVL FLip Texture
GLvoid glmFlipTexture(unsigned char* texture, int width, int height)
{
    int pixcount = 0;
    unsigned char* buf;
    buf = (unsigned char*)malloc(sizeof(unsigned char)*width*3);

    for (int i = 0; i < height/2; i++)
	{
	    memcpy (buf, (&texture[3*width*(height-1-i)]), 3*width*sizeof(unsigned char));
	    memcpy ((&texture[3*width*(height-1-i)]), (&texture[3*width*i]), 3*width*sizeof(unsigned char));
	    memcpy ((&texture[3*width*i]), buf, 3*width*sizeof(unsigned char));
	}
    free(buf);
}
//AVL END Flip Texture 


//AVL Flip Model Textures
GLvoid glmFlipModelTextures(GLMmodel* model)
{
    static GLuint i;
    static GLMgroup* group;
    static GLMmaterial* material;
    
    assert(model);
    assert(model->vertices);

    /* perhaps this loop should be unrolled into material, color, flat,
       smooth, etc. loops?  since most cpu's have good branch prediction
       schemes (and these branches will always go one way), probably
       wouldn't gain too much?  */
    
    group = model->groups;
    while (group) {
	material = &model->materials[group->material];
        
        //AVL Texture Flip
        if (material->image) {
            glmFlipTexture(material->image, material->width, material->height);                    	
            
            glBindTexture(_glmTextureTarget, model->materials[group->material].t_id[0]);
            //glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	    //glTexImage2D(_glmTextureTarget, 0, GL_RGB, model->materials[nummaterials].width,
            //             model->materials[nummaterials].height, 0, GL_RGBA, GL_UNSIGNED_BYTE, model->materials[nummaterials]->image);
            gluBuild2DMipmaps(_glmTextureTarget, 3, model->materials[group->material].width, model->materials[group->material].height,
                              GL_RGB, GL_UNSIGNED_BYTE,  model->materials[group->material].image);
	    //glTexParameterf(_glmTextureTarget, GL_TEXTURE_MIN_FILTER, GL_LINEAR); 
	    //glTexParameterf(_glmTextureTarget, GL_TEXTURE_MAG_FILTER, GL_LINEAR); 
        }
       
        group = group->next;
    }

}
//AVL END Flip Model Textures
#endif

#if 0
/* normals */
if (model->numnormals) {
    numvectors = model->numnormals;
    vectors  = model->normals;
    copies = glmOptimizeVectors(vectors, &numvectors);
    
    __glmWarning("glmOptimize(): %d redundant normals.", 
		 model->numnormals - numvectors);
    
    for (i = 0; i < model->numtriangles; i++) {
        T(i).nindices[0] = (GLuint)vectors[3 * T(i).nindices[0] + 0];
        T(i).nindices[1] = (GLuint)vectors[3 * T(i).nindices[1] + 0];
        T(i).nindices[2] = (GLuint)vectors[3 * T(i).nindices[2] + 0];
    }
    
    /* free space for old normals */
    free(vectors);
    
    /* allocate space for the new normals */
    model->numnormals = numvectors;
    model->normals = (GLfloat*)malloc(sizeof(GLfloat) * 
				      3 * (model->numnormals + 1));
    
    /* copy the optimized vertices into the actual vertex list */
    for (i = 1; i <= model->numnormals; i++) {
        model->normals[3 * i + 0] = copies[3 * i + 0];
        model->normals[3 * i + 1] = copies[3 * i + 1];
        model->normals[3 * i + 2] = copies[3 * i + 2];
    }
    
    free(copies);
}

/* texcoords */
if (model->numtexcoords) {
    numvectors = model->numtexcoords;
    vectors  = model->texcoords;
    copies = glmOptimizeVectors(vectors, &numvectors);
    
    __glmWarning("glmOptimize(): %d redundant texcoords.", 
		 model->numtexcoords - numvectors);
    
    for (i = 0; i < model->numtriangles; i++) {
        for (j = 0; j < 3; j++) {
            T(i).tindices[j] = (GLuint)vectors[3 * T(i).tindices[j] + 0];
        }
    }
    
    /* free space for old texcoords */
    free(vectors);
    
    /* allocate space for the new texcoords */
    model->numtexcoords = numvectors;
    model->texcoords = (GLfloat*)malloc(sizeof(GLfloat) * 
					2 * (model->numtexcoords + 1));
    
    /* copy the optimized vertices into the actual vertex list */
    for (i = 1; i <= model->numtexcoords; i++) {
        model->texcoords[2 * i + 0] = copies[2 * i + 0];
        model->texcoords[2 * i + 1] = copies[2 * i + 1];
    }
    
    free(copies);
}
#endif

#if 0
/* look for unused vertices */
/* look for unused normals */
/* look for unused texcoords */
for (i = 1; i <= model->numvertices; i++) {
    for (j = 0; j < model->numtriangles; i++) {
        if (T(j).vindices[0] == i || 
            T(j).vindices[1] == i || 
            T(j).vindices[1] == i)
            break;
    }
}
#endif
