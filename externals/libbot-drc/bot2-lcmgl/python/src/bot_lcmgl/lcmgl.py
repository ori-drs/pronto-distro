import cStringIO as StringIO
import bot_lcmgl.data_t as data_t
import struct

LCMGL_GL_BEGIN         = 4
LCMGL_GL_END           = 5
LCMGL_GL_VERTEX3F      = 6
LCMGL_GL_VERTEX3D      = 7
LCMGL_GL_COLOR3F       = 8
LCMGL_GL_COLOR4F       = 9
LCMGL_GL_POINTSIZE     = 10
LCMGL_GL_ENABLE        = 11
LCMGL_GL_DISABLE       = 12
LCMGL_BOX              = 13  # nyi
LCMGL_CIRCLE           = 14
LCMGL_GL_LINE_WIDTH    = 15
LCMGL_NOP              = 16
LCMGL_GL_VERTEX2D      = 17
LCMGL_GL_VERTEX2F      = 18
LCMGL_TEXT             = 19
LCMGL_DISK             = 20
LCMGL_GL_TRANSLATED    = 21
LCMGL_GL_ROTATED       = 22
LCMGL_GL_LOAD_IDENTITY = 23
LCMGL_GL_PUSH_MATRIX   = 24
LCMGL_GL_POP_MATRIX    = 25
LCMGL_RECT             = 26  # nyi
LCMGL_TEXT_LONG        = 27
LCMGL_GL_NORMAL3F      = 28
LCMGL_GL_SCALEF        = 29
LCMGL_GL_MULT_MATRIXF  = 30  # nyi
LCMGL_GL_MULT_MATRIXD  = 31  # nyi
LCMGL_GL_MATERIALF     = 32
LCMGL_GL_PUSH_ATTRIB   = 33  # nyi
LCMGL_GL_POP_ATTRIB    = 34  # nyi
LCMGL_GL_DEPTH_FUNC    = 35  # nyi
LCMGL_TEXTURE2D        = 36
LCMGL_TEXTURE_DRAW_QUAD= 37
LCMGL_SPHERE           = 38
LCMGL_CYLINDER         = 39

# text flags
LCMGL_TEXT_DROP_SHADOW                   = 1
LCMGL_TEXT_JUSTIFY_LEFT                  = 2
LCMGL_TEXT_JUSTIFY_RIGHT                 = 4
LCMGL_TEXT_JUSTIFY_CENTER                = 8
LCMGL_TEXT_ANCHOR_LEFT                   = 16
LCMGL_TEXT_ANCHOR_RIGHT                  = 32
LCMGL_TEXT_ANCHOR_TOP                    = 64
LCMGL_TEXT_ANCHOR_BOTTOM                 = 128
LCMGL_TEXT_ANCHOR_HCENTER                = 256
LCMGL_TEXT_ANCHOR_VCENTER                = 512
LCMGL_TEXT_NORMALIZED_SCREEN_COORDINATES = 1024
LCMGL_TEXT_MONOSPACED                    = 2048

# texture constants
LCMGL_LUMINANCE = 0
LCMGL_RGB = 1
LCMGL_RGBA = 2

LCMGL_COMPRESS_NONE = 0

# redefine OpenGL constants
GL_POINTS         = 0x0000
GL_LINES          = 0x0001
GL_LINE_LOOP      = 0x0002
GL_LINE_STRIP     = 0x0003
GL_TRIANGLES      = 0x0004
GL_TRIANGLE_STRIP = 0x0005
GL_TRIANGLE_FAN   = 0x0006
GL_QUADS          = 0x0007
GL_QUAD_STRIP     = 0x0008
GL_POLYGON        = 0x0009

GL_NEVER             = 0x0200
GL_LESS              = 0x0201
GL_EQUAL             = 0x0202
GL_LEQUAL            = 0x0203
GL_GREATER           = 0x0204
GL_NOTEQUAL          = 0x0205
GL_GEQUAL            = 0x0206
GL_ALWAYS            = 0x0207
GL_DEPTH_TEST        = 0x0B71
GL_DEPTH_BITS        = 0x0D56
GL_DEPTH_CLEAR_VALUE = 0x0B73
GL_DEPTH_FUNC        = 0x0B74
GL_DEPTH_RANGE       = 0x0B70
GL_DEPTH_WRITEMASK   = 0x0B72
GL_DEPTH_COMPONENT   = 0x1902

GL_LIGHTING = 0xb50

GL_FRONT = 0x0404
GL_BACK = 0x0405

GL_AMBIENT = 0x1200
GL_DIFFUSE = 0x1201
GL_SPECULAR = 0x1202
GL_SHININESS = 0x1601
GL_EMISSION = 0x1600

for n in range(1, 9):
    args = ", ".join(["a%d" % i for i in range(n)])
    exec """
def _lcmgl_make_encode_%d(cmd, fmt):
    st = struct.Struct(">B%%s" %% fmt)
    def encode(self, %s):
        self.data.write(st.pack(cmd, %s))
        self.datalen += st.size
    return encode
""" % (n, args, args)

def _lcmgl_make_encode_0(cmd):
    data = struct.pack("B", cmd)
    def encode(self):
        self.data.write(data)
        self.datalen += 1
    return encode
    
class lcmgl:
    def __init__(self, name, lcm):
        self.lcm = lcm
        self.data = StringIO.StringIO()
        self.datalen = 0
        self.scene = 1
        self.name = name
        self.ntextures = 0

    def switch_buffer(self):
        d = self.data.getvalue()

        msg = data_t()
        msg.name = self.name
        msg.scene = self.scene
        msg.sequence = 0
        msg.datalen = len(d)
        msg.data = d
        self.lcm.publish("LCMGL", msg.encode())

        self.ntextures = 0
        self.data = StringIO.StringIO()
        self.scene += 1

    glBegin        = _lcmgl_make_encode_1(LCMGL_GL_BEGIN, "I")
    glEnd          = _lcmgl_make_encode_0(LCMGL_GL_END)

    glVertex2f     = _lcmgl_make_encode_2(LCMGL_GL_VERTEX2F, "ff")
    glVertex2d     = _lcmgl_make_encode_2(LCMGL_GL_VERTEX2D, "dd")
    glVertex3f     = _lcmgl_make_encode_3(LCMGL_GL_VERTEX3F, "fff")
    glVertex3d     = _lcmgl_make_encode_3(LCMGL_GL_VERTEX3D, "ddd")

    glColor3f      = _lcmgl_make_encode_3(LCMGL_GL_COLOR3F, "fff")
    glColor4f      = _lcmgl_make_encode_4(LCMGL_GL_COLOR4F, "ffff")

    glPointSize    = _lcmgl_make_encode_1(LCMGL_GL_POINTSIZE, "f")
    glLineWidth    = _lcmgl_make_encode_1(LCMGL_GL_LINE_WIDTH, "f")

    glEnable       = _lcmgl_make_encode_1(LCMGL_GL_ENABLE, "I")
    glDisable      = _lcmgl_make_encode_1(LCMGL_GL_DISABLE, "I")

    glTranslated   = _lcmgl_make_encode_3(LCMGL_GL_TRANSLATED, "ddd")
    glRotated      = _lcmgl_make_encode_4(LCMGL_GL_ROTATED, "dddd")
    glScalef       = _lcmgl_make_encode_3(LCMGL_GL_SCALEF, "fff")

    glLoadIdentity = _lcmgl_make_encode_0(LCMGL_GL_LOAD_IDENTITY)
    glPushMatrix   = _lcmgl_make_encode_0(LCMGL_GL_PUSH_MATRIX)
    glPopMatrix    = _lcmgl_make_encode_0(LCMGL_GL_POP_MATRIX)

    glNormal3f     = _lcmgl_make_encode_3(LCMGL_GL_NORMAL3F, "fff")
    glMaterialf    = _lcmgl_make_encode_6(LCMGL_GL_MATERIALF, "IIffff")

    # circle(x, y, z, radius)
    circle         = _lcmgl_make_encode_4(LCMGL_CIRCLE, "dddf")
    # disk(x, y, z, r_in, r_out)
    disk           = _lcmgl_make_encode_5(LCMGL_DISK, "dddff")
    # sphere(x, y, z, radius, slices, stacks)
    sphere         = _lcmgl_make_encode_6(LCMGL_SPHERE, "ddddII")
    # cylinder(x, y, z, r_base, r_top, height, slices, stacks)
    cylinder       = _lcmgl_make_encode_8(LCMGL_CYLINDER, "ddddddII")

    def text(self, x, y, z, text, flags = 0):
        font = 0
        self.data.write(struct.pack(">BIIdddI", LCMGL_TEXT_LONG, font, flags, x, y, z, len(text)))
        self.data.write(text)
        self.datalen += 37 + len(text)

    def texture2d(self, data, width, height, format, compression):
        self.ntextures += 1
        tex_id = self.ntextures

        if format not in [ LCMGL_LUMINANCE, LCMGL_RGB, LCMGL_RGBA ]:
            raise ValueError("Invalid format")
#        bytes_per_pixel = 1
#        if format == LCMGL_LUMINANCE:
#            bytes_per_pixel = 1
#        elif format == LCMGL_RGB:
#            bytes_per_pixel = 3
#        elif format == LCMGL_RGBA:
#            bytes_per_pixel = 4
#        else:
#        bytes_per_row = width * bytes_per_pixel
#        datalen = bytes_per_row * height

        if compression == LCMGL_COMPRESS_NONE:
            data_tosend = data
        else:
            raise ValueError("Invalid compression value")

        self.data.write(struct.pack(">BIIIIII", LCMGL_TEXTURE2D, tex_id,
            width, height, format, compression, len(data_tosend)))
        self.data.write(data_tosend)

        return tex_id

    def textureDrawQuad(self, tex_id, 
            top_left_xyz, bot_left_xyz,
            bot_right_xyz, top_right_xyz):

        if tex_id > self.ntextures or tex_id <= 0:
            raise ValueError("Invalid texture ID")
        self.data.write(struct.pack(">BIdddddddddddd", LCMGL_TEXTURE_DRAW_QUAD,
            tex_id,
            top_left_xyz[0], top_left_xyz[1], top_left_xyz[2], 
            bot_left_xyz[0], bot_left_xyz[1], bot_left_xyz[2], 
            bot_right_xyz[0], bot_right_xyz[1], bot_right_xyz[2], 
            top_right_xyz[0], top_right_xyz[1], top_right_xyz[2]))


if __name__ == "__main__":
    import array
    import math
    import lcm
    lcm = lcm.LCM()
    a = array.array('B')
    width = 100
    height = 100
    g = lcmgl('lcmgl_texture', lcm)
    for y in range(height):
        for x in range(width):
            v = math.sin(x / 5.) + math.cos(y / 5.)
            a.append(int(v * 50 + 127))
    img_data = a.tostring()
    print len(img_data)
    tex_id = g.texture2d(img_data, width, height, LCMGL_LUMINANCE, LCMGL_COMPRESS_NONE)
    g.glColor3f(0, 0, 1)
    g.textureDrawQuad(tex_id, 
            (-10, 10, 0),
            (-10, -10, 0),
            (10, -10, 0),
            (10, 10, 0))
    g.switch_buffer()

    import time
    g = lcmgl('lcmgl_test', lcm)
    for i in range(90):
        g.glLineWidth(1 + i / 25.0)
        g.glColor3f(1, 1, 0)

        px, py = 0, 0
        nx, ny = 0, 0

        g.glBegin(GL_LINES)
        for j in range(i):
            nx = 0.1 * j * math.cos(j / 5.)
            ny = 0.1 * j * math.sin(j / 5.)
            g.glVertex3d(px, py, 0)
            g.glVertex3d(nx, ny, 0)
            px, py = nx, ny
        g.glEnd()

        g.glEnable(GL_LIGHTING)
        g.glMaterialf(GL_FRONT, GL_AMBIENT, 0, 0.7, 0, 1);
        g.glMaterialf(GL_FRONT, GL_DIFFUSE, 0, 0.7, 0, 1);

        g.sphere(nx, ny, 0, 1, 10, 10)

        g.switch_buffer()
        time.sleep(0.05)
