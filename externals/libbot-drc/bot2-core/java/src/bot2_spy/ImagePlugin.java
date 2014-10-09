package bot2_spy;

import java.util.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.*;
import java.awt.geom.*;
import java.io.*;
import javax.imageio.*;
import javax.swing.*;
import javax.swing.event.*;

import lcm.spy.ChannelData;
import lcm.lcm.LCMDataInputStream;

import bot_core.image_t;

/** A plugin for viewing bot_core.image_t data **/
public class ImagePlugin implements lcm.spy.SpyPlugin
{
    public boolean canHandle(long fingerprint)
    {
        return fingerprint == bot_core.image_t.LCM_FINGERPRINT;
    }

    class MyAction extends AbstractAction
    {
        ChannelData cd;
        JDesktopPane jdp;

        public MyAction(JDesktopPane jdp, ChannelData cd)
        {
            super("Image Viewer");
            this.jdp = jdp;
            this.cd = cd;
        }

        public void actionPerformed(ActionEvent e) 
        {
            Viewer v = new Viewer(cd);
            jdp.add(v);
            v.toFront();
        }
    }

    public Action getAction(JDesktopPane jdp, ChannelData cd)
    {
        return new MyAction(jdp, cd);
    }

    class Viewer extends JInternalFrame implements lcm.lcm.LCMSubscriber
    {
        ChannelData cd;
        JImage ji;

        public Viewer(ChannelData cd)
        {
            super("Image: "+cd.name, true, true);
            this.cd = cd;

            setLayout(new BorderLayout());
            ji = new JImage(null, true);
            add(ji, BorderLayout.CENTER);
            setSize(400,300);
            setVisible(true);

            lcm.lcm.LCM.getSingleton().subscribe(cd.name, this);
        }

        BufferedImage handleRAW(image_t v)
        {
            BufferedImage bi = new BufferedImage(v.width, v.height, 
                    BufferedImage.TYPE_INT_RGB);
            for (int y = 0; y < v.height; y++) {
                for (int x = 0; x < v.width; x++) {
                    int g = v.data[x+y*v.row_stride] & 0xff;
                    int rgb = (g<<16)|(g<<8)|g;
                    bi.setRGB(x, y, rgb);
                }
            }
            return bi;
        }

        BufferedImage handleRGB(image_t v)
        {
            BufferedImage bi = new BufferedImage(v.width, v.height, 
                    BufferedImage.TYPE_INT_RGB);
            for (int y = 0; y < v.height; y++) {
                for (int x = 0; x < v.width; x++) {
                    int r = v.data[y*v.row_stride + x*3 + 0] & 0xff;
                    int g = v.data[y*v.row_stride + x*3 + 1] & 0xff;
                    int b = v.data[y*v.row_stride + x*3 + 2] & 0xff;
                    int rgb = (r<<16)|(g<<8)|b;
                    bi.setRGB(x, y, rgb);
                }
            }
            return bi;
        }

        BufferedImage handleJPEG(image_t v)
        {
            try {
                return ImageIO.read(new ByteArrayInputStream(v.data));
            } catch (IOException ex) {
                return null;
            }
        }

        public void handleImage(image_t v)
        {
            if (v.width==0 || v.height==0)
                return;

            BufferedImage bi = null;

            switch (v.pixelformat) 
            {
                case image_t.PIXEL_FORMAT_MJPEG:
                    bi = handleJPEG(v);
                    break;
                case image_t.PIXEL_FORMAT_GRAY:
                case image_t.PIXEL_FORMAT_BAYER_BGGR:
                case image_t.PIXEL_FORMAT_BAYER_GBRG:
                case image_t.PIXEL_FORMAT_BAYER_GRBG:
                case image_t.PIXEL_FORMAT_BAYER_RGGB:
                    bi = handleRAW(v);
                    break;
                case image_t.PIXEL_FORMAT_RGB:
                    bi = handleRGB(v);
                    break;
            }

            if(bi != null)
                ji.setImage(bi);
        }

        public void messageReceived (lcm.lcm.LCM lc, 
                String channel, LCMDataInputStream ins)
        {
            try {
                image_t v = new image_t(ins);
                handleImage(v);
            } catch (IOException ex) {
                System.out.println("ex: "+ex);
                return;
            }
        }
    }

    static class JImage extends JComponent
    {
        public static final long serialVersionUID=1001;

        BufferedImage im;
        boolean fit = false;
        JPopupMenu popupMenu = new JPopupMenu("JImage Menu");
        AffineTransform t = new AffineTransform();

        public JImage()
        {
            this(null, true);
        }

        public JImage(int width, int height)
        {
            this(new BufferedImage(width, height, BufferedImage.TYPE_BYTE_GRAY), true);
        }


        public JImage(BufferedImage im)
        {
            this(im, true);
        }

        public JImage(BufferedImage im, boolean enableControls)
        {
            this.im = im;

            if (enableControls)
                initControls();
        }

        void initControls()
        {
            JMenuItem jmi;

            jmi=new JMenuItem("JImage Menu");
            jmi.setEnabled(false);
            popupMenu.add(jmi);

            jmi=new JMenuItem("Fit");
            jmi.addActionListener(new ActionListener() {
                public void actionPerformed(ActionEvent e) {
                    fit = true; repaint();
                }
            });
            popupMenu.add(jmi);

            JMenu scaleMenu = new JMenu("Set Scale");
            popupMenu.add(scaleMenu);
            int scales[] = new int[] { 25, 50, 100, 200, 400, 800 };

            for (int i = 0; i < scales.length; i++)
            {
                jmi=new JMenuItem(scales[i]+" %");
                jmi.addActionListener(new ScaleAction(scales[i]));
                scaleMenu.add(jmi);
            }


            MyListener l = new MyListener();
            addMouseMotionListener(l);
            addMouseListener(l);
            addMouseWheelListener(l);
            addKeyListener(l);
        }

        class ScaleAction implements ActionListener
        {
            int scale;

            public ScaleAction(int scale)
            {
                this.scale = scale;
            }

            public void actionPerformed(ActionEvent e)
            {
                adjustZoom(scale/100.0, null);
            }
        }

        /** Zoom, preserving the center point on the screen. (When we
          draw, the center point may be moved in order to maximize the
          amount of image seen on screen.) **/
        void adjustZoom(double z, Point2D cp)
        {
            if (cp == null)
                cp = new Point2D.Double(getWidth()/2, getHeight()/2);

            Point2D cxt = componentToImage(cp);
            double tx = cp.getX() - cxt.getX()*z;
            double ty = cp.getY() - cxt.getY()*z;

            t = new AffineTransform();
            t.translate(tx, ty);
            t.scale(z, z);

            fit = false;
            repaint();
        }

        // zoom in a notch
        void zoomIn(Point2D p)
        {
            double nz = t.getScaleX()*2.0;
            nz = Math.min(nz, 64);

            adjustZoom(nz, p);
        }

        // zoom out a notch
        void zoomOut(Point2D p)
        {
            double nz = t.getScaleX()*.5;
            nz = Math.max(nz, 1.0/32.0);

            adjustZoom(nz, p);
        }

        // if im == null, the image is cleared
        public void setImage(BufferedImage im)
        {
            this.im=im;
            invalidate();
            repaint();
        }

        public Dimension getMinimumSize()
        {
            //	if (im==null)
            return new Dimension(2,2);

            //	return new Dimension(im.getWidth(), im.getHeight());
        }

        public Dimension getPreferredSize()
        {
            return getMaximumSize();
        }

        public Dimension getMaximumSize()
        {
            if (im==null)
                return new Dimension(2,2);

            return new Dimension(im.getWidth(), im.getHeight());
        }

        public Point2D componentToImage(Point2D p)
        {
            Point2D tp = null;

            try {
                tp = t.inverseTransform(p, null);
            } catch (NoninvertibleTransformException ex) {
                // can't happen.
            }

            return tp;
        }

        public synchronized void paint(Graphics gin)
        {
            Graphics2D g=(Graphics2D) gin;

            if (im==null)
                return;

            int height=getHeight();
            int width=getWidth();

            if (fit)
            {
                t = new AffineTransform();
                double scale = Math.min(((double) width)/im.getWidth(), 
                        ((double) height)/im.getHeight());
                // we'll re-center the transform in a moment.
                t.scale(scale, scale);
            }

            // if the image (in either X or Y) is smaller than the view port, then center
            // the image with respect to that axis.
            double mwidth = im.getWidth()*t.getScaleX();
            double mheight = im.getHeight()*t.getScaleY();
            if (mwidth < width)
                t.preConcatenate(AffineTransform.getTranslateInstance( (width-mwidth) / 2.0 - t.getTranslateX(), 0) );
            if (mheight < height)
                t.preConcatenate(AffineTransform.getTranslateInstance( 0, (height-mheight)/2.0 - t.getTranslateY()));

            // if we're allowing panning (because only a portion of the image is visible),
            // don't allow translations that show less information that is possible.
            Point2D topleft     = t.transform(new Point2D.Double(0,0), null);
            Point2D bottomright = t.transform(new Point2D.Double(im.getWidth(),im.getHeight()), null);

            if (mwidth > width)
            {
                if (topleft.getX() > 0)
                    t.preConcatenate(AffineTransform.getTranslateInstance(-topleft.getX(),0));
                if (bottomright.getX() < width)
                    t.preConcatenate(AffineTransform.getTranslateInstance(width-bottomright.getX(), 0));
                //		    t.translate(width-bottomright.getX(), 0);
            }
            if (mheight > height)
            {
                if (topleft.getY() > 0)
                    t.preConcatenate(AffineTransform.getTranslateInstance(0, -topleft.getY()));
                if (bottomright.getY() < height)
                    t.preConcatenate(AffineTransform.getTranslateInstance(0, height-bottomright.getY()));
            }

            g.drawImage(im, t, null);
        }

        /*    public void setScale(boolean b)
              {
              scale=b;
              }
              */

        class MyListener implements KeyListener, 
              MouseMotionListener, 
              MouseListener, MouseWheelListener
              {
                  boolean     nodrag     = true;
                  Point       dragBegin  = null;

                  public void keyPressed(KeyEvent e)
                  {
                  }

                  public void keyReleased(KeyEvent e)
                  {
                  }

                  public void keyTyped(KeyEvent e)
                  {
                  }

                  // returns true if the keypress was handled
                  public boolean handleKeyPress(KeyEvent e)
                  {
                      return false;
                  }

                  public void mouseWheelMoved(MouseWheelEvent e)
                  {
                      int amount=e.getWheelRotation();

                      Point2D p = new Point2D.Double(e.getPoint().getX(), e.getPoint().getY());

                      if (amount > 0)
                          zoomOut(p);
                      if (amount < 0)
                          zoomIn(p);
                  }

                  public void mouseDragged(MouseEvent e)
                  {
                      int mods=e.getModifiersEx();
                      Point dragEnd = e.getPoint();
                      boolean shift = (mods&MouseEvent.SHIFT_DOWN_MASK)>0;
                      boolean ctrl = (mods&MouseEvent.CTRL_DOWN_MASK)>0;
                      boolean alt = shift & ctrl;
                      ctrl = ctrl & (!alt);
                      shift = shift & (!alt);
                      boolean nomods = !(shift | ctrl | alt);

                      if (dragBegin == null)
                          dragBegin = dragEnd;

                      nodrag=false;

                      if ((mods&InputEvent.BUTTON3_DOWN_MASK)>0 || true)
                      {
                          double dx = dragEnd.getX() - dragBegin.getX();
                          double dy = dragEnd.getY() - dragBegin.getY();

                          synchronized (JImage.this)
                          {
                              t.preConcatenate(AffineTransform.getTranslateInstance(dx, dy));
                          }

                          dragBegin = dragEnd;
                          repaint();
                      }
                  }

                  public void mouseMoved(MouseEvent e)
                  {
                  }

                  public void mousePressed(MouseEvent e)
                  {
                      dragBegin = e.getPoint();
                  }

                  public void mouseReleased(MouseEvent e)
                  {
                      int mods=e.getModifiersEx();

                      if (nodrag && e.getButton()==MouseEvent.BUTTON3)
                      {
                          popupMenu.show(JImage.this, (int) e.getPoint().getX(), (int) e.getPoint().getY());
                      }

                      nodrag = true;
                      dragBegin = null;
                  }

                  public void mouseClicked(MouseEvent e)
                  {
                      if (e.getButton()==1 && e.getClickCount()==2)
                      {
                          adjustZoom(1, null);
                      }
                  }

                  public void mouseEntered(MouseEvent e)
                  {
                  }

                  public void mouseExited(MouseEvent e)
                  {
                  }
              }
    }
}
