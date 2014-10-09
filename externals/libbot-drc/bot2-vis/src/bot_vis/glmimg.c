/*
 * Adapted from F. Devernay's extensions to Nate Robbins' GLM library
 *
 * Source obtained from GLM-0.3.1 available from http://devernay.free.fr/hacks/glm/
 *
 * Changes:
 *     [mwalter, May 23, 2011]: Simplified to support use of PPM, JPEG (via LIBJPEG),
 *                              and PNG (via LIBPNG)
 *
 *
 */
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

# ifdef _WIN32
#   include <windows.h>
# endif
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>
#else
#include <GL/gl.h>
#include <GL/glext.h>
#endif
#include "glm.h"
/*
#define DEBUG
#define GLDEBUG
*/
#include "glmint.h"



/* WARNING: GLOBAL VARIABLES */
GLenum _glmTextureTarget = GL_TEXTURE_2D;
static GLint gl_max_texture_size;
static int glm_do_init = 1;
static GLboolean gl_sgis_generate_mipmap = GL_FALSE;

static GLboolean glmIsExtensionSupported(const char *extension)
{
    
    const GLubyte *extensions = NULL;
    const GLubyte *start;
    GLubyte *where, *terminator;
    
    /* Extension names should not have spaces. */
    where = (GLubyte *) strchr(extension, ' ');
    if (where || *extension == '\0')
        return 0;
    
    extensions = glGetString(GL_EXTENSIONS);
    if (!extensions)
        return GL_FALSE;
    
    /* It takes a bit of care to be fool-proof about parsing the
       OpenGL extensions string.  Don't be fooled by sub-strings,
       etc. */
    start = extensions;
    for (;;) {
        where = (GLubyte *) strstr((const char *) start, extension);
        if (!where)
            break;
        terminator = where + strlen(extension);
        if (where == start || *(where - 1) == ' ')
            if (*terminator == ' ' || *terminator == '\0')
                return GL_TRUE;
        start = terminator;
    }
    return GL_FALSE;
}

static void glmImgInit(void)
{
    glm_do_init = 0;
    _glmTextureTarget = GL_TEXTURE_2D;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &gl_max_texture_size);
#if GLM_MAX_TEXTURE_SIZE > 0    
#warning GLM_MAX_TEXTURE_SIZE
    if(gl_max_texture_size > GLM_MAX_TEXTURE_SIZE)
        gl_max_texture_size = GLM_MAX_TEXTURE_SIZE;
#endif
    //return;
#if 0				/* rectangle textures */
#ifdef GL_TEXTURE_RECTANGLE_ARB
    if (glmIsExtensionSupported("GL_ARB_texture_rectangle")) {
        DBG_(__glmWarning("glmImgInit(): GL_ARB_texture_rectangle is available"));
        _glmTextureTarget = GL_TEXTURE_RECTANGLE_ARB;
    }
    else
#endif
#ifdef GL_TEXTURE_RECTANGLE_NV
        if (glmIsExtensionSupported("GL_NV_texture_rectangle")) {
            DBG_(__glmWarning("glmImgInit(): GL_NV_texture_rectangle is available"));
            _glmTextureTarget = GL_TEXTURE_RECTANGLE_NV;
        }
#endif
#endif				/* rectangle textures */
#ifdef GL_GENERATE_MIPMAP_SGIS
    if (glmIsExtensionSupported("GL_SGIS_generate_mipmap")) {
        DBG_(__glmWarning("glmImgInit(): GL_SGIS_generate_mipmap is available"));
        gl_sgis_generate_mipmap = GL_TRUE;
    }
#endif
    /*_glmTextureTarget = GL_TEXTURE_2D;*/
}

/* glmReadPPM: read a PPM raw (type P6) file.  The PPM file has a header
 * that should look something like:
 *
 *    P6
 *    # comment
 *    width height max_value
 *    rgbrgbrgb...
 *
 * where "P6" is the magic cookie which identifies the file type and
 * should be the only characters on the first line followed by a
 * carriage return.  Any line starting with a # mark will be treated
 * as a comment and discarded.   After the magic cookie, three integer
 * values are expected: width, height of the image and the maximum
 * value for a pixel (max_value must be < 256 for PPM raw files).  The
 * data section consists of width*height rgb triplets (one byte each)
 * in binary format (i.e., such as that written with fwrite() or
 * equivalent).
 *
 * The rgb data is returned as an array of unsigned chars (packed
 * rgb).  The malloc()'d memory should be free()'d by the caller.  If
 * an error occurs, an error message is sent to stderr and NULL is
 * returned.
 *
 * filename   - name of the .ppm file.
 * width      - will contain the width of the image on return.
 * height     - will contain the height of the image on return.
 *
 */
static GLubyte* 
glmReadPPM(const char* filename, GLboolean alpha, int* width, int* height, int *type)
{
    FILE* fp;
    int i, w, h, d;
    unsigned char* image;
    char head[70];          /* max line <= 70 in PPM (per spec). */
    
    fp = fopen(filename, "rb");
    if (!fp) {
        perror(filename);
        return NULL;
    }
    
    /* grab first two chars of the file and make sure that it has the
       correct magic cookie for a raw PPM file. */
    fgets(head, 70, fp);
    if (strncmp(head, "P6", 2)) {
        DBG_(__glmWarning("glmReadPPM() failed: %s: Not a raw PPM file", filename));
        return NULL;
    }
    
    /* grab the three elements in the header (width, height, maxval). */
    i = 0;
    while(i < 3) {
        fgets(head, 70, fp);
        if (head[0] == '#')     /* skip comments. */
            continue;
        if (i == 0)
            i += sscanf(head, "%d %d %d", &w, &h, &d);
        else if (i == 1)
            i += sscanf(head, "%d %d", &h, &d);
        else if (i == 2)
            i += sscanf(head, "%d", &d);
    }
    
    /* grab all the image data in one fell swoop. */
    image = (unsigned char*)malloc(sizeof(unsigned char)*w*h*3);
    fread(image, sizeof(unsigned char), w*h*3, fp);
    fclose(fp);
    
    *type = GL_RGB;
    *width = w;
    *height = h;
    return image;
}




/* don't try alpha=GL_FALSE: gluScaleImage implementations seem to be buggy */
GLuint
glmLoadTexture(const char *filename, GLboolean alpha, GLboolean repeat, GLboolean filtering, 
               GLboolean mipmaps, GLfloat *texcoordwidth, GLfloat *texcoordheight)
{
    GLuint tex;
    int width, height,pixelsize;
    int type;
    int filter_min, filter_mag;
    GLubyte *data, *rdata;
    double xPow2, yPow2;
    int ixPow2, iyPow2;
    int xSize2, ySize2;
    GLint retval;
    
    if(glm_do_init)
        glmImgInit();
    
    /* fallback solution (PPM only) */
    data = glmReadPPM(filename, alpha, &width, &height, &type);
    if(data != NULL) {
        DBG_(__glmWarning("glmLoadTexture(): got PPM for %s",filename));
        goto DONE;
    }


    // Assuming that we have libjpeg
    data = glmReadJPG(filename, alpha, &width, &height, &type);
    if(data != NULL) {
        DBG_(__glmWarning("glmLoadTexture(): got JPG for %s",filename));
        goto DONE;
    }

    // Assuming that we have libpng
    data = glmReadPNG(filename, alpha, &width, &height, &type);
    if(data != NULL) {
        DBG_(__glmWarning("glmLoadTexture(): got PNG for %s",filename));
        goto DONE;
    }
    
    __glmWarning("glmLoadTexture() failed: Unable to load texture from %s!", filename);
    DBG_(__glmWarning("glmLoadTexture() failed: tried PPM"));
    
    DBG_(__glmWarning("glmLoadTexture() failed: tried JPEG"));
    
    return 0;
    
 DONE:

   // Give up if the maximum texture size is 0
    if (gl_max_texture_size == 0) {
        DBG_(__glmWarning("glmLoadTexture(): Maximum texture size is %d. Skipping texture!",
                          gl_max_texture_size));
        
        return 0;
    }

    /*#define FORCE_ALPHA*/
#ifdef FORCE_ALPHA
    if(alpha && type == GL_RGB) {
        /* if we really want RGBA */
        const unsigned int size = width * height;
        
        unsigned char *rgbaimage;
        unsigned char *ptri, *ptro;
        int i;
        
        rgbaimage = (unsigned char*)malloc(sizeof(unsigned char)* size * 4);
        ptri = data;
        ptro = rgbaimage;
        for(i=0; i<size; i++) {
            *(ptro++) = *(ptri++);
            *(ptro++) = *(ptri++);
            *(ptro++) = *(ptri++);
            *(ptro++) = 255;
        }
        free(data);
        data = rgbaimage;
        type = GL_RGBA;
    }
#endif /* FORCE_ALPHA */
    switch(type) {
    case GL_LUMINANCE:
        pixelsize = 1;
        break;
    case GL_RGB:
    case GL_BGR:
        pixelsize = 3;
        break;
    case GL_RGBA:
    case GL_BGRA:
        pixelsize = 4;
        break;
    default:
        __glmFatalError( "glmLoadTexture(): unknown type 0x%x", type);
        pixelsize = 0;
        break;
    }
    
    if((pixelsize*width) % 4 == 0)
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    else
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    
    xSize2 = width;
    if (xSize2 > gl_max_texture_size)
        xSize2 = gl_max_texture_size;
    ySize2 = height;
    if (ySize2 > gl_max_texture_size)
        ySize2 = gl_max_texture_size;
    
    if (_glmTextureTarget == GL_TEXTURE_2D) {
        //if(1) {
        /* scale image to power of 2 in height and width */
        xPow2 = log((double)xSize2) / log(2.0);
        yPow2 = log((double)ySize2) / log(2.0);
        
        ixPow2 = (int)xPow2;
        iyPow2 = (int)yPow2;
        
        if (xPow2 != (double)ixPow2)
            ixPow2++;
        if (yPow2 != (double)iyPow2)
            iyPow2++;
        
        xSize2 = 1 << ixPow2;
        ySize2 = 1 << iyPow2;
    }
    
    DBG_(__glmWarning("gl_max_texture_size=%d / width=%d / xSize2=%d / height=%d / ySize2 = %d", gl_max_texture_size, width, xSize2, height, ySize2));
    if((width != xSize2) || (height != ySize2)) {
        /* TODO: use glTexSubImage2D instead */
        DBG_(__glmWarning("scaling texture"));
        rdata = (GLubyte*)malloc(sizeof(GLubyte) * xSize2 * ySize2 * pixelsize);
        if (!rdata)
            return 0;
	    
        retval = gluScaleImage(type, width, height,
                               GL_UNSIGNED_BYTE, data,
                               xSize2, ySize2, GL_UNSIGNED_BYTE,
                               rdata);
        
        free(data);
        data = rdata;
    }
    
    glGenTextures(1, &tex);		/* Generate texture ID */
    glBindTexture(_glmTextureTarget, tex);
    DBG_(__glmWarning("building texture %d",tex));
    
    if(mipmaps && _glmTextureTarget != GL_TEXTURE_2D) {
        DBG_(__glmWarning("mipmaps only work with GL_TEXTURE_2D"));
        mipmaps = 0;
    }
    if(filtering) {
        filter_min = (mipmaps) ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR;
        filter_mag = GL_LINEAR;
    }
    else {
        filter_min = (mipmaps) ? GL_NEAREST_MIPMAP_NEAREST : GL_NEAREST;
        filter_mag = GL_NEAREST;
    }
    glTexParameteri(_glmTextureTarget, GL_TEXTURE_MIN_FILTER, filter_min);
    glTexParameteri(_glmTextureTarget, GL_TEXTURE_MAG_FILTER, filter_mag);
    
    glTexParameteri(_glmTextureTarget, GL_TEXTURE_WRAP_S, (repeat) ? GL_REPEAT : GL_CLAMP);
    glTexParameteri(_glmTextureTarget, GL_TEXTURE_WRAP_T, (repeat) ? GL_REPEAT : GL_CLAMP);
    if(mipmaps && _glmTextureTarget == GL_TEXTURE_2D) {
        /* only works for GL_TEXTURE_2D */
#ifdef GL_GENERATE_MIPMAP_SGIS
        if(gl_sgis_generate_mipmap) {
            DBG_(__glmWarning("sgis mipmapping"));
            glTexParameteri(_glmTextureTarget, GL_GENERATE_MIPMAP_SGIS, GL_TRUE );
            glTexImage2D(_glmTextureTarget, 0, type, xSize2, ySize2, 0, type, 
                         GL_UNSIGNED_BYTE, data);
        }
        else
#endif
            {
                DBG_(__glmWarning("glu mipmapping"));
                gluBuild2DMipmaps(_glmTextureTarget, type, xSize2, ySize2, type, 
                                  GL_UNSIGNED_BYTE, data);
            }
    }
    else {
        glTexImage2D(_glmTextureTarget, 0, type, xSize2, ySize2, 0, type, 
                     GL_UNSIGNED_BYTE, data);
    }
    
    
    /* Clean up and return the texture ID */
    free(data);
    
    if (_glmTextureTarget == GL_TEXTURE_2D) {
        *texcoordwidth = 1.;		/* texcoords are in [0,1] */
        *texcoordheight = 1.;
    }
    else {
        *texcoordwidth = xSize2;		/* size of texture coords */
        *texcoordheight = ySize2;
    }
    
    return tex;
}
