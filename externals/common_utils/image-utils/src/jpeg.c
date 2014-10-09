#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <jpeglib.h>
#include <jerror.h>

#include "pixels.h"
#include "jpeg.h"

static void
init_source (j_decompress_ptr cinfo)
{
}

static boolean
fill_input_buffer (j_decompress_ptr cinfo)
{
//    fprintf (stderr, "Error: JPEG decompressor ran out of buffer space\n");
    return TRUE;
}

static void
skip_input_data (j_decompress_ptr cinfo, long num_bytes)
{
    cinfo->src->next_input_byte += num_bytes;
    cinfo->src->bytes_in_buffer -= num_bytes;
}

static void
term_source (j_decompress_ptr cinfo)
{
}

static void 
jpeg_err_emit_message(j_common_ptr cinfo, int msg_level)
{
    // suppress warnings and errors
}

static int
jpeg_decompress_8u (const uint8_t * src, int src_size,
        uint8_t * dest, int width, int height, int stride, J_COLOR_SPACE ocs);

int 
jpeg_decompress_8u_rgb (const uint8_t * src, int src_size,
        uint8_t * dest, int width, int height, int stride)
{
    return jpeg_decompress_8u (src, src_size, dest, width, height,
            stride, JCS_RGB);
}

int 
jpeg_decompress_8u_gray (const uint8_t * src, int src_size,
        uint8_t * dest, int width, int height, int stride)
{
    return jpeg_decompress_8u (src, src_size, dest, width, height,
            stride, JCS_GRAYSCALE);
}

int
jpeg_decompress_8u (const uint8_t * src, int src_size,
        uint8_t * dest, int width, int height, int stride, J_COLOR_SPACE ocs)
{
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    struct jpeg_source_mgr jsrc;

    cinfo.err = jpeg_std_error (&jerr);
    jerr.emit_message = jpeg_err_emit_message;
    jpeg_create_decompress (&cinfo);

    jsrc.next_input_byte = src;
    jsrc.bytes_in_buffer = src_size;
    jsrc.init_source = init_source;
    jsrc.fill_input_buffer = fill_input_buffer;
    jsrc.skip_input_data = skip_input_data;
    jsrc.resync_to_restart = jpeg_resync_to_restart;
    jsrc.term_source = term_source;
    cinfo.src = &jsrc;

    jpeg_read_header (&cinfo, TRUE);
    cinfo.out_color_space = ocs;
    jpeg_start_decompress (&cinfo);

    if (cinfo.output_height != height || cinfo.output_width != width) {
        fprintf (stderr, "Error: Buffer was %dx%d but JPEG image is %dx%d\n",
                width, height, cinfo.output_width, cinfo.output_height);
        jpeg_destroy_decompress (&cinfo);
        return -1;
    }

    while (cinfo.output_scanline < height) {
        uint8_t * row = dest + cinfo.output_scanline * stride;
        jpeg_read_scanlines (&cinfo, &row, 1);
    }
    jpeg_finish_decompress (&cinfo);
    jpeg_destroy_decompress (&cinfo);
    return 0;
}

static void
init_destination (j_compress_ptr cinfo)
{
    /* do nothing */
}

static boolean
empty_output_buffer (j_compress_ptr cinfo)
{
    fprintf (stderr, "Error: JPEG compressor ran out of buffer space\n");
    return TRUE;
}

static void
term_destination (j_compress_ptr cinfo)
{
    /* do nothing */
}

int
jpeg_compress_8u_gray (const uint8_t * src, int width, int height, int stride,
        uint8_t * dest, int * destsize, int quality)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    struct jpeg_destination_mgr jdest;
    int out_size = *destsize;

    cinfo.err = jpeg_std_error (&jerr);
    jpeg_create_compress (&cinfo);
    jdest.next_output_byte = dest;
    jdest.free_in_buffer = out_size;
    jdest.init_destination = init_destination;
    jdest.empty_output_buffer = empty_output_buffer;
    jdest.term_destination = term_destination;
    cinfo.dest = &jdest;

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 1;
    cinfo.in_color_space = JCS_GRAYSCALE;
    jpeg_set_defaults (&cinfo);
    jpeg_set_quality (&cinfo, quality, TRUE);

    jpeg_start_compress (&cinfo, TRUE);
    while (cinfo.next_scanline < height) {
        JSAMPROW row = (JSAMPROW)(src + cinfo.next_scanline * stride);
        jpeg_write_scanlines (&cinfo, &row, 1);
    }
    jpeg_finish_compress (&cinfo);
    *destsize = out_size - jdest.free_in_buffer;
    jpeg_destroy_compress (&cinfo);
    return 0;
}

int
jpeg_compress_8u_rgb (const uint8_t * src, int width, int height, int stride,
        uint8_t * dest, int * destsize, int quality)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    struct jpeg_destination_mgr jdest;
    int out_size = *destsize;

    cinfo.err = jpeg_std_error (&jerr);
    jpeg_create_compress (&cinfo);
    jdest.next_output_byte = dest;
    jdest.free_in_buffer = out_size;
    jdest.init_destination = init_destination;
    jdest.empty_output_buffer = empty_output_buffer;
    jdest.term_destination = term_destination;
    cinfo.dest = &jdest;

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults (&cinfo);
    jpeg_set_quality (&cinfo, quality, TRUE);

    jpeg_start_compress (&cinfo, TRUE);
    while (cinfo.next_scanline < height) {
        JSAMPROW row = (JSAMPROW)(src + cinfo.next_scanline * stride);
        jpeg_write_scanlines (&cinfo, &row, 1);
    }
    jpeg_finish_compress (&cinfo);
    *destsize = out_size - jdest.free_in_buffer;
    jpeg_destroy_compress (&cinfo);
    return 0;
}

int
jpeg_compress_8u_bgra (const uint8_t * src, int width, int height, int stride,
        uint8_t * dest, int * destsize, int quality)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    struct jpeg_destination_mgr jdest;
    int out_size = *destsize;

    cinfo.err = jpeg_std_error (&jerr);
    jpeg_create_compress (&cinfo);
    jdest.next_output_byte = dest;
    jdest.free_in_buffer = out_size;
    jdest.init_destination = init_destination;
    jdest.empty_output_buffer = empty_output_buffer;
    jdest.term_destination = term_destination;
    cinfo.dest = &jdest;

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults (&cinfo);
    jpeg_set_quality (&cinfo, quality, TRUE);

    jpeg_start_compress (&cinfo, TRUE);
    while (cinfo.next_scanline < height) {
        uint8_t buf[width*3];
        pixel_convert_8u_bgra_to_8u_rgb (buf, 0, width, 1,
                src + cinfo.next_scanline * stride, 0);
        JSAMPROW row = (JSAMPROW) buf;
        jpeg_write_scanlines (&cinfo, &row, 1);
    }
    jpeg_finish_compress (&cinfo);
    *destsize = out_size - jdest.free_in_buffer;
    jpeg_destroy_compress (&cinfo);
    return 0;
}

#if 0
int
jpeg_decompress_8u_rgb_IPP (const uint8_t * src, int src_size,
        uint8_t * dest, int width, int height, int stride)
{
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    struct jpeg_source_mgr jsrc;

    cinfo.err = jpeg_std_error (&jerr);
    jpeg_create_decompress (&cinfo);

    jsrc.next_input_byte = src;
    jsrc.bytes_in_buffer = src_size;
    jsrc.init_source = init_source;
    jsrc.fill_input_buffer = fill_input_buffer;
    jsrc.skip_input_data = skip_input_data;
    jsrc.resync_to_restart = jpeg_resync_to_restart;
    jsrc.term_source = term_source;
    cinfo.src = &jsrc;

    jpeg_read_header (&cinfo, TRUE);
    cinfo.out_color_space = JCS_RGB;
    jpeg_calc_output_dimensions (&cinfo);

    if (cinfo.output_height < height || cinfo.output_width < width) {
        fprintf (stderr, "Error: Buffer is %dx%d but JPEG image is %dx%d\n",
                width, height, cinfo.output_width, cinfo.output_height);
        jpeg_destroy_decompress (&cinfo);
        return -1;
    }

    jvirt_barray_ptr * dct_planes = jpeg_read_coefficients (&cinfo);

    uint8_t * tmp_plane = NULL;
    uint8_t * full_plane[cinfo.num_components];
    int full_stride;
    int tmp_stride;
    int max_h_samp=1, max_v_samp=1;
    int c, i, j;
    jpeg_component_info * c0 = cinfo.cur_comp_info[0];
    for (c = 0; c < cinfo.num_components; c++) {
        full_plane[c] = ippiMalloc_8u_C1 (c0->width_in_blocks*8,
                c0->height_in_blocks*8, &full_stride);
        jpeg_component_info * comp = cinfo.cur_comp_info[c];
        if (comp->h_samp_factor > max_h_samp)
            max_h_samp = comp->h_samp_factor;
        if (comp->v_samp_factor > max_v_samp)
            max_v_samp = comp->v_samp_factor;
    }
    for (c = 0; c < cinfo.num_components; c++) {
        jpeg_component_info * comp = cinfo.cur_comp_info[c];
        uint8_t * plane;
        int pstride, offset;
        int h_samp = max_h_samp / comp->h_samp_factor;
        int v_samp = max_v_samp / comp->v_samp_factor;
        if (h_samp == 2 && v_samp == 2) {
            if (!tmp_plane)
                tmp_plane = ippiMalloc_8u_C1 (comp->width_in_blocks*8 + 16,
                        comp->height_in_blocks*8 + 2, &tmp_stride);
            plane = tmp_plane;
            pstride = tmp_stride;
            offset = tmp_stride + 8;
        }
        else if (h_samp == 1 && v_samp == 1) {
            plane = full_plane[c];
            pstride = full_stride;
            offset = 0;
        }
        for (i = 0; i < comp->height_in_blocks; i++) {
            JBLOCKARRAY bar = cinfo.mem->access_virt_barray (
                    (j_common_ptr) &cinfo, dct_planes[c],
                    i, 1, FALSE);
            JBLOCKROW row = bar[0];
            uint8_t * drow = plane + 8*i*pstride + offset;
            for (j = 0; j < comp->width_in_blocks; j++) {
                JBLOCK * blk = row + j;
                ippiDCTQuantInv8x8LS_JPEG_16s8u_C1R (*blk, drow + j*8,
                        pstride,
                        cinfo.quant_tbl_ptrs[comp->quant_tbl_no]->quantval);
            }
        }
        if (h_samp == 2 && v_samp == 2) {
            IppiSize srcroi = { 8*comp->width_in_blocks,
                8*comp->height_in_blocks };
            IppiSize dstroi = { 8*comp->width_in_blocks + 2,
                8*comp->height_in_blocks + 2 };
            ippiCopyReplicateBorder_8u_C1IR (tmp_plane + tmp_stride + 8,
                    tmp_stride, srcroi, dstroi, 1, 1);
            dstroi.width = width;
            dstroi.height = height;
            ippiSampleUpH2V2_JPEG_8u_C1R (tmp_plane + tmp_stride + 8,
                    tmp_stride, srcroi, full_plane[c], full_stride, dstroi);
        }
    }
    IppiSize dstroi = { width, height };
    ippiYCbCrToRGB_JPEG_8u_P3C3R ((const uint8_t **)full_plane,
            full_stride, dest, stride, dstroi);
    for (c = 0; c < cinfo.num_components; c++)
        ippiFree (full_plane[c]);
    if (tmp_plane)
        ippiFree (tmp_plane);

    jpeg_finish_decompress (&cinfo);
    jpeg_destroy_decompress (&cinfo);
    return 0;
}

int
jpeg_compress_8u_rgb_IPP (const uint8_t * src, int width, int height,
        int stride, uint8_t * dest, int * destsize, int quality)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    struct jpeg_destination_mgr jdest;
    int out_size = *destsize;

    cinfo.err = jpeg_std_error (&jerr);
    jpeg_create_compress (&cinfo);
    jdest.next_output_byte = dest;
    jdest.free_in_buffer = out_size;
    jdest.init_destination = init_destination;
    jdest.empty_output_buffer = empty_output_buffer;
    jdest.term_destination = term_destination;
    cinfo.dest = &jdest;

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults (&cinfo);
    jpeg_set_quality (&cinfo, quality, TRUE);


    jvirt_barray_ptr barptr[3];
    for (c = 0; c < cinfo.num_components; c++) {
        jpeg_component_info * comp = cinfo.cur_comp_info[c];
        barptr[c] = cinfo.mem->request_virt_barray (&cinfo, JPOOL_IMAGE,
                FALSE, cinfo->width_in_blocks, cinfo->height_in_blocks,
                cinfo->height_in_blocks);
    }

    jpeg_write_coefficients (&cinfo, barptr);

    for (c = 0; c < cinfo.num_components; c++) {
        jpeg_component_info * comp = cinfo.cur_comp_info[c];
        uint8_t * plane;
        int pstride;
        int h_samp = max_h_samp / comp->h_samp_factor;
        int v_samp = max_v_samp / comp->v_samp_factor;
        if (h_samp == 2 && v_samp == 2) {
            if (!tmp_plane)
                tmp_plane = ippiMalloc_8u_C1 (comp->width_in_blocks*8,
                        comp->height_in_blocks*8, &tmp_stride);
            plane = tmp_plane;
            pstride = tmp_stride;
        }
        else if (h_samp == 1 && v_samp == 1) {
            plane = full_plane[c];
            pstride = full_stride;
        }
        for (i = 0; i < comp->height_in_blocks; i++) {
            JBLOCKARRAY bar = cinfo.mem->access_virt_barray (
                    (j_common_ptr) &cinfo, dct_planes[c],
                    i, 1, TRUE);
            JBLOCKROW row = bar[0];
            uint8_t * srow = plane + 8*i*pstride;
            for (j = 0; j < comp->width_in_blocks; j++) {
                JBLOCK * blk = row + j;
                ippiDCTQuantFwd8x8LS_JPEG_8u16s_C1R (srow + j*8,
                        pstride, *blk,
                        cinfo.quant_tbl_ptrs[comp->quant_tbl_no]->quantval);
            }
        }
        if (h_samp == 2 && v_samp == 2) {
            IppiSize srcroi = { 8*comp->width_in_blocks,
                8*comp->height_in_blocks };
            IppiSize dstroi = { 8*comp->width_in_blocks + 2,
                8*comp->height_in_blocks + 2 };
            ippiCopyReplicateBorder_8u_C1IR (tmp_plane + tmp_stride + 8,
                    tmp_stride, srcroi, dstroi, 1, 1);
            dstroi.width = width;
            dstroi.height = height;
            ippiSampleUpH2V2_JPEG_8u_C1R (tmp_plane + tmp_stride + 8,
                    tmp_stride, srcroi, full_plane[c], full_stride, dstroi);
        }
    }
#if 0
    jpeg_start_compress (&cinfo, TRUE);
    while (cinfo.next_scanline < height) {
        JSAMPROW row = (JSAMPROW)(src + cinfo.next_scanline * stride);
        jpeg_write_scanlines (&cinfo, &row, 1);
    }
#endif
    jpeg_finish_compress (&cinfo);
    *destsize = out_size - jdest.free_in_buffer;
    jpeg_destroy_compress (&cinfo);
    return 0;
}

#endif


int
jpeg_get_dimensions (const uint8_t * src, int src_size, int *width, int *height)
{
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    struct jpeg_source_mgr jsrc;

    cinfo.err = jpeg_std_error (&jerr);
    jpeg_create_decompress (&cinfo);

    jsrc.next_input_byte = src;
    jsrc.bytes_in_buffer = src_size;
    jsrc.init_source = init_source;
    jsrc.fill_input_buffer = fill_input_buffer;
    jsrc.skip_input_data = skip_input_data;
    jsrc.resync_to_restart = jpeg_resync_to_restart;
    jsrc.term_source = term_source;
    cinfo.src = &jsrc;

    int ret = jpeg_read_header (&cinfo, TRUE);
    //    printf ("jpeg_read_header: %d\n", ret);
    cinfo.out_color_space = JCS_RGB;
    ret = jpeg_start_decompress (&cinfo);
    //    printf ("jpeg_start_decompress: %d\n", ret);

    *width = cinfo.output_width;
    *height = cinfo.output_height;
    //    *cs = cinfo.out_color_space; //TODO: should get the color space info...
    jpeg_destroy_decompress (&cinfo);
    return 0;
}
