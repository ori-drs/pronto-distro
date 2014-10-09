/*
===========================================================================

Project:   Generic Polygon Clipper

           A new algorithm for calculating the difference, intersection,
           exclusive-or or union of arbitrary polygon sets.

           Note: minor modifications by dcm to reuse some existing data
           structures from the DGC code base.

File:      gpc.h
Author:    Alan Murta (email: gpc@cs.man.ac.uk)
Version:   2.32
Date:      17th December 2004

Copyright: (C) 1997-2004, Advanced Interfaces Group,
           University of Manchester.

           This software is free for non-commercial use. It may be copied,
           modified, and redistributed provided that this copyright notice
           is preserved on all copies. The intellectual property rights of
           the algorithms used reside with the University of Manchester
           Advanced Interfaces Group.

           You may not use this software, in whole or in part, in support
           of any commercial product without the express consent of the
           author.

           There is no warranty or other guarantee of fitness of this
           software for any purpose. It is provided solely "as is".

===========================================================================
*/

#ifndef __gpc_h
#define __gpc_h

#include <stdio.h>
#include "geometry.h"

#ifdef __cplusplus
extern "C" {
#endif


/*
===========================================================================
                               Constants
===========================================================================
*/

/* Increase GPC_EPSILON to encourage merging of near coincident edges    */

#define GPC_EPSILON (DBL_EPSILON)

#define GPC_VERSION "2.32"


/*
===========================================================================
                           Public Data Types
===========================================================================
*/

typedef enum                        /* Set operation type                */
{
  GPC_DIFF,                         /* Difference                        */
  GPC_INT,                          /* Intersection                      */
  GPC_XOR,                          /* Exclusive or                      */
  GPC_UNION                         /* Union                             */
} gpc_op;

#if 0
typedef struct                      /* Polygon vertex structure          */
{
  double              x;            /* Vertex x component                */
  double              y;            /* vertex y component                */
} gpc_vertex;

typedef struct                      /* Vertex list structure             */
{
  int                 num_vertices; /* Number of vertices in list        */
  gpc_vertex         *vertex;       /* Vertex array pointer              */
} gpc_vertex_list;
#endif

typedef point2d_t gpc_vertex;
typedef pointlist2d_t gpc_vertex_list;

typedef struct                      /* Polygon set structure             */
{
  int                 num_contours; /* Number of contours in polygon     */
  int                *hole;         /* Hole / external contour flags     */
  gpc_vertex_list    *contour;      /* Contour array pointer             */
} gpc_polygon;


/*
===========================================================================
                       Public Function Prototypes
===========================================================================
*/

void gpc_read_polygon        (FILE            *infile_ptr, 
                              int              read_hole_flags,
                              gpc_polygon     *polygon);

void gpc_write_polygon       (FILE            *outfile_ptr,
                              int              write_hole_flags,
                              gpc_polygon     *polygon);

void gpc_add_contour         (gpc_polygon     *polygon,
                              gpc_vertex_list *contour,
                              int              hole);

void gpc_polygon_clip        (gpc_op           set_operation,
                              const gpc_polygon     *subject_polygon,
                              const gpc_polygon     *clip_polygon,
                              gpc_polygon     *result_polygon);

void gpc_free_polygon        (gpc_polygon     *polygon);

#ifdef __cplusplus
}
#endif

#endif

/*
===========================================================================
                           End of file: gpc.h
===========================================================================
*/
