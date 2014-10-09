#ifndef __glmint_h__
#define __glmint_h__

extern GLenum _glmTextureTarget;

/* private routines from glm_util.c */
extern char * __glmStrStrip(const char *string);
#ifdef HAVE_STRDUP
#define __glmStrdup strdup
#else
extern char * __glmStrdup(const char *string);
#endif
extern void __glmWarning(char *format,...);
extern void __glmFatalError(char *format,...);
extern void __glmFatalUsage(char *format,...);
extern char* __glmDirName(char* path);
void __glmReportErrors(void);

#ifdef DEBUG
#define DBG_(_x)       ((void)(_x))
#else
#define DBG_(_x)       ((void)0)
#endif

#ifdef GLDEBUG
#define GLDBG_(_x)       { GLenum ret = (_x); if(ret != GL_NO_ERROR) __glmWarning("OpenGL error at %d : %s",__LINE__,gluErrorString(ret)); }
#else
#define GLDBG_(_x)       ((void)0)
#endif

#ifndef GL_BGR
#define GL_BGR GL_BGR_EXT
#endif

#ifndef GL_BGRA
#define GL_BGRA GL_BGRA_EXT
#endif


GLubyte* glmReadDevIL(const char*, GLboolean, int*, int*, int*);
GLubyte* glmReadJPG(const char*, GLboolean, int*, int*, int*);
GLubyte* glmReadPNG(const char*, GLboolean, int*, int*, int*);
GLubyte* glmReadSDL(const char*, GLboolean, int*, int*, int*);
GLubyte* glmReadSimage(const char*, GLboolean, int*, int*, int*);
#endif /* __glmint_h__ */
