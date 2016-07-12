#include <qt4/QtGui/QPainter>
#include <qt4/QtGui/QImageWriter>

int main(int argc, char **argv)
{
  QImage img(argv[1]);
  QPainter painter(&img);

  painter.setPen(qRgb(0,255,0)); //color to draw with
  painter.drawLine(QPoint(img.width()/4,img.height()/2), QPoint(3*img.width()/4,img.height()/2));// similarly drawPoint, drawPolyogon etc. function exists.

  QImageWriter imgw("img_qt.jpg", "jpg");
  imgw.write(img);
}
