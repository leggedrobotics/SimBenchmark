//
// Created by jhwangbo on 18.11.16.
//

#ifndef RAI_GNUPLOTFIGUREMANAGER_HPP
#define RAI_GNUPLOTFIGUREMANAGER_HPP

#include "gnuplot_cpp.hpp"
#include "raiCommon/utils/rai_timer/RAI_timer_items.hpp"
#include <vector>
#include "raiCommon/utils/rai_message_logger/rai_message.hpp"

namespace rai {
namespace Utils {
namespace Graph {

class GnuplotFigureManager {

 public:

  GnuplotFigureManager() { }
  ~GnuplotFigureManager() {
    for (auto const &figure: figures_)
      delete (figure);
    system("rm -rf /tmp/gnuplot*");
  }

  void figure(int figureNumber, FigProp2D &properties) {
    RAIWARN_IF(figureNumber>5 || figureNumber<0,"valid figure number between 0 and 5");
    figures_[figureIdx(figureNumber)]->figure(figureNumber, properties);
  };

  void figure3D(int figureNumber, FigProp3D &properties) {
    RAIWARN_IF(figureNumber>5 || figureNumber<0,"valid figure number between 0 and 5");
    figures_[figureIdx(figureNumber)]->figure3D(figureNumber, properties);
  }

  /* drawing a heatmap for grid data */
  template<typename Dtype>
  void drawHeatMap(int figureNumber,
                   FigProp3D &properties,
                   Dtype *x,
                   Dtype *y,
                   Dtype *z,
                   int xCount,
                   int yCount,
                   std::string name) {
    RAIWARN_IF(figureNumber>5 || figureNumber<0,"valid figure number between 0 and 5");
    figures_[figureIdx(figureNumber)]->drawHeatMap(figureNumber, properties, x, y, z, xCount, yCount, name);
  }

  /* Only outputs to the screen*/
  void drawFigure(int figureNumber) {
    RAIWARN_IF(figureNumber>5 || figureNumber<0,"valid figure number between 0 and 5");
    figures_[figureIdx(figureNumber)]->drawFigure();
  }

  /*Output to the screen and to a file as well*/
  void drawFigure(int figureNumber, OutputFormat format, bool saveAllFigures = false) {
    RAIWARN_IF(figureNumber>5 || figureNumber<0,"valid figure number between 0 and 5");
    figures_[figureIdx(figureNumber)]->drawFigure(format, saveAllFigures, log_path_);
  }

  /* adding a curve to a graph */
  template<typename Dtype>
  void appendData(int figureNumber, Dtype *x, Dtype *y, int dataLen, std::string name) {
    RAIWARN_IF(figureNumber>5 || figureNumber<0,"valid figure number between 0 and 5");
    figures_[figureIdx(figureNumber)]->appendData(x, y, dataLen, name);
  }

  template<typename Dtype>
  void appendData(int figureNumber, Eigen::VectorXd& x, Eigen::VectorXd& y, std::string name) {
    RAIWARN_IF(figureNumber>5 || figureNumber<0,"valid figure number between 0 and 5");
    figures_[figureIdx(figureNumber)]->appendData(x.data(), y.data(), x.cols(), name);
  }

  /* adding a curve to a graph
    Plotting Methods:
    dots, lines, linespoints, arrows, points, impulses, steps
    Style: lt(line type), lc(line color), lw(line width), pt(point type), ps(point size), pi(point interval)

    ex) plot.appendData(x_data.data(), y_data.data(), 201, "linespoints", "test", "lt -1 lc 10 lw 2 pt 7 ps 1.0 pi 10")
    ex) plot.appendData(x_data.data(), y_data.data(), 201, "linespoints", "test", "lt -1 lc \"green\" lw 2")
    !!for color, line&point types, check http://stelweb.asu.cas.cz/~nemeth/work/stuff/gnuplot/gnuplot-line-and-point-types.png
   */
  template<typename Dtype>
  void appendData(int figureNumber, Dtype *x,
                  Dtype *y,
                  int dataLen,
                  PlotMethods2D plottingMethod,
                  std::string name,
                  std::string style) {
    RAIWARN_IF(figureNumber>5 || figureNumber<0,"valid figure number between 0 and 5");
    figures_[figureIdx(figureNumber)]->appendData(x, y, dataLen, plottingMethod, name, style);
  }

  template<typename Dtype>
  void appendData(int figureNumber,
                  Eigen::VectorXd& x,
                  Eigen::VectorXd& y,
                  PlotMethods2D plottingMethod,
                  std::string name,
                  std::string style) {
    RAIWARN_IF(figureNumber>5 || figureNumber<0,"valid figure number between 0 and 5");
    figures_[figureIdx(figureNumber)]->appendData(x.data(), y.data(), x.rows(), plottingMethod, name, style);
  }

  template<typename Dtype>
  void appendArrows(int figureNumber,
                    Dtype *startingX,
                    Dtype *startingY,
                    Dtype *startingZ,
                    Dtype *arrowX,
                    Dtype *arrowY,
                    Dtype *arrowZ,
                    int dataLen,
                    std::string name,
                    std::string style) {
    RAIWARN_IF(figureNumber>5 || figureNumber<0,"valid figure number between 0 and 5");
    figures_[figureIdx(figureNumber)]->appendArrows(startingX,
                                                    startingY,
                                                    startingZ,
                                                    arrowX,
                                                    arrowY,
                                                    arrowZ,
                                                    dataLen,
                                                    name,
                                                    style);
  }

  template<typename Dtype>
  void appendArrows(int figureNumber,
                    Dtype *startingX,
                    Dtype *startingY,
                    Dtype *arrowX,
                    Dtype *arrowY,
                    int dataLen,
                    std::string name,
                    std::string style) {
    RAIWARN_IF(figureNumber>5 || figureNumber<0,"valid figure number between 0 and 5");
    figures_[figureIdx(figureNumber)]->appendArrows(startingX, startingY, arrowX, arrowY, dataLen, name, style);
  }

  /* adding a curve and standard deviation to a graph
    Plotting Methods:
    dots, lines, linespoints, arrows, points, impulses, steps
    Style: lt(line type), lc(line color), lw(line width), pt(point type), ps(point size), pi(point interval)

    ex) plot.appendData(x_data.data(), y_data.data(), 201, "linespoints", "test", "lt -1 lc 10 lw 2 pt 7 ps 1.0 pi 10")
    ex) plot.appendData(x_data.data(), y_data.data(), 201, "linespoints", "test", "lt -1 lc \"green\" lw 2")
    !!for color, line&point types, check http://stelweb.asu.cas.cz/~nemeth/work/stuff/gnuplot/gnuplot-line-and-point-types.png
    */
  template<typename Dtype>
  void appendDataAndStd(int figureNumber,
                        Dtype *x,
                        Dtype *y,
                        Dtype *std,
                        int dataLen,
                        std::string name,
                        std::string style = "") {
    RAIWARN_IF(figureNumber>5 || figureNumber<0,"valid figure number between 0 and 5");
    figures_[figureIdx(figureNumber)]->appendDataAndStd(x, y, std, dataLen, name, style);
  }

  /*
  heatOn: value of z is also represented by color
 */
  template<typename Dtype>
  void append3D_Data(int figureNumber,
                     Dtype *x,
                     Dtype *y,
                     Dtype *z,
                     int dataLen,
                     bool heatOn,
                     PlotMethods3D method,
                     std::string name,
                     std::string style = "") {
    RAIWARN_IF(figureNumber>5 || figureNumber<0,"valid figure number between 0 and 5");
    figures_[figureIdx(figureNumber)]->append3D_Data(x, y, z, dataLen, heatOn, method, name, style);
  }

  /*No figure(), no drawFigure()*/
  template<typename Dtype>
  void drawPieChart(int figureNumber,
                    const std::vector<Dtype>& time,
                    const std::vector<std::string>& name,
                    std::string unit,
                    FigPropPieChart& prop) {
    RAIWARN_IF(figureNumber>5 || figureNumber<0,"valid figure number between 0 and 5");
    figures_[figureIdx(figureNumber)]->drawPieChart(figureNumber, time, name, unit, prop);
  }

 /*No figure(), no drawFigure()*/
  void drawPieChartWith_RAI_Timer(int figureNumber,
                                  Timer_items &items,
                                  FigPropPieChart& prop) {
   RAIWARN_IF(figureNumber>5 || figureNumber<0,"valid figure number between 0 and 5");
    figures_[figureIdx(figureNumber)]->drawPieChartWithSubCate(figureNumber, items, prop);
  }

  /// it clears tempfiles as well
  void waitForEnter() {
    std::cout << "Press enter to exit." << std::endl;
    std::cin.get();
  }

  void setLogPath(std::string& path){
    log_path_ = path;
  }

 private:

  int figureIdx(int figureNumber) {
    unsigned int idx = std::find(figureID_.begin(), figureID_.end(), figureNumber) - figureID_.begin();
    if (idx == figureID_.size()) {
      figures_.push_back(new Figure(figureNumber));
      figureID_.push_back(figureNumber);
    }
    return idx;
  }

  std::vector<Figure *> figures_;
  std::vector<int> figureID_;
  std::string log_path_;

};

}
}
}

#endif //RAI_GNUPLOTFIGUREMANAGER_HPP
