//
// Written by Jemin Hwangbo, RSL, ETHZ, Switzerland on 19.07.16.
//

#ifndef RAI_GNUPLOT_CPP_HPP
#define RAI_GNUPLOT_CPP_HPP

// standard inclusions
#include <cstdlib>
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <memory>
#include <algorithm>
#include <Eigen/Core>
#include "raiCommon/utils/rai_timer/RAI_timer_items.hpp"
#include <math.h>
#include <algorithm>
#include <iomanip>      // std::setprecision
//#include "gnuplotFigureManager.hpp"

namespace rai {
namespace Utils {
namespace Graph {

enum class PlotMethods2D : unsigned int {
  dots = 0,
  lines,
  linespoints,
  arrows,
  points
};

enum class PlotMethods3D : unsigned int {
  dots = 0,
  lines,
  points
};

enum class DisplayType3D {
  heatMap2D,
  points3D,
  heatMap3D
};

enum class OutputFormat {
  eps,
  png,
  pdf,
  svg
};

struct FigProp2D {
 public:
  FigProp2D(){}
  FigProp2D(std::string xlabel_, std::string ylabel_, std::string title_) {
    xlabel = xlabel_; ylabel = ylabel_; title = title_;
  }
  std::string xlabel = "", ylabel = "", title = "", size = "600,400";
};

/*
palette examples:
rgb 7,5,15; "traditional pm3d\n(black-blue-red-yellow)";
rgb 3,11,6; "green-red-violet"
rgb 23,28,3; "ocean (green-blue-white)"
rgb 21,22,23; "hot (black-red-yellow-white)"
rgb 30,31,32; "color printable on gray\n(black-blue-violet-yellow-white)"
rgb 33,13,10; "rainbow (blue-green-yellow-red)"
rgb 34,35,36; "AFM hot (black-red-yellow-white)"
*/
struct FigProp3D {
 public:
  FigProp3D(){}
  FigProp3D(std::string xlabel_, std::string ylabel_, std::string zlabel_, std::string title_) {
    xlabel = xlabel_; ylabel = ylabel_; title = title_; zlabel = zlabel_;
  }
  std::string xlabel = "x axis", ylabel = "y axis", zlabel = "z axis", title = "No title", palette = "rgb 33,13,10",
      gridLineColor = "black", size = "600,400";
  DisplayType3D displayType = DisplayType3D::heatMap2D;
};

struct FigPropPieChart {
 public:
  std::string title = "No title", palette = "rgb 33,13,10", size = "600,400";
};


class Figure {
  constexpr static auto termType = "qt";
  constexpr static auto GNUPLOT = "/usr/bin/gnuplot";
 private:

  enum class FigureType {
    figureType2D,
    figureType3D,
    figureMultiPlot,
    No_figure
  };

 public:
  Figure(int ID) {
    figID = ID;
    pipe_ = popen(GNUPLOT, "w");

    plottingMethodsEnumToStringMap_.insert({PlotMethods2D::dots, "dots"});
    plottingMethodsEnumToStringMap_.insert({PlotMethods2D::lines, "lines"});
    plottingMethodsEnumToStringMap_.insert({PlotMethods2D::linespoints, "linespoints"});
    plottingMethodsEnumToStringMap_.insert({PlotMethods2D::arrows, "arrows"});
    plottingMethodsEnumToStringMap_.insert({PlotMethods2D::points, "points"});
    plottingMethodsEnumToStringMap_.insert({PlotMethods2D::dots, "impulses"});
    plottingMethodsEnumToStringMap_.insert({PlotMethods2D::dots, "steps"});
    plottingMethods3DEnumToStringMap_.insert({PlotMethods3D::lines, "lines"});
    plottingMethods3DEnumToStringMap_.insert({PlotMethods3D::dots, "dots"});
    plottingMethods3DEnumToStringMap_.insert({PlotMethods3D::points, "points"});

    OutputFormatStringMap_.insert({OutputFormat::png, "png"});
    OutputFormatStringMap_.insert({OutputFormat::pdf, "pdf"});
    OutputFormatStringMap_.insert({OutputFormat::eps, "eps"});
    OutputFormatStringMap_.insert({OutputFormat::svg, "svg"});

  }

  ~Figure() {
  }

  /* initialize a figure */
  void figure(int figureNumber, FigProp2D &properties) {

    std::string size_string="";

    if(!properties.size.empty())
      size_string = "size " + properties.size;

    firstCurve_ = true;
    figureType_ = FigureType::figureType2D;
    fprintf(pipe_, "unset colorbox\n");
    fprintf(pipe_, "set style fill transparent solid 0.3\n");

    if (!multiPlotOn_) fprintf(pipe_, "set terminal %s %i position %s %s\n", termType, figID, position_[figureNumber].c_str(), size_string.c_str());
    fprintf(pipe_, "set title \"%s\" \n", properties.title.c_str());
    fprintf(pipe_, "set xlabel \"%s\" \n", properties.xlabel.c_str());
    fprintf(pipe_, "set ylabel \"%s\" \n", properties.ylabel.c_str());
    fprintf(pipe_, "set title \"%s\" \n", properties.title.c_str());
    fprintf(pipe_, "plot");
  }

  void figure3D(int figureNumber, FigProp3D &properties) {
    std::string size_string="";

    if(!properties.size.empty())
      size_string = "size " + properties.size;

    firstCurve_ = true;
    fprintf(pipe_, "unset view\n");
    fprintf(pipe_, "unset pm3d\n");
    fprintf(pipe_, "set colorbox\n");
    fprintf(pipe_, "set style fill transparent solid 0.8\n");

    figureType_ = FigureType::figureType3D;

    if (!multiPlotOn_) fprintf(pipe_, "set terminal %s %i position %s %s\n", termType, figID, position_[figureNumber].c_str(), size_string.c_str());
    if (!properties.palette.empty()) fprintf(pipe_, "set palette %s\n", properties.palette.c_str());
    fprintf(pipe_, "set xlabel \"%s\" \n", properties.xlabel.c_str());
    fprintf(pipe_, "set ylabel \"%s\" \n", properties.ylabel.c_str());
    fprintf(pipe_, "set zlabel \"%s\" rotate left \n", properties.zlabel.c_str());
    fprintf(pipe_, "set title \"%s\" \n", properties.title.c_str());

    fprintf(pipe_, "splot");
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
    assert_false(properties.displayType == DisplayType3D::points3D, "Only heatMap2D and heatMap3D are allowed!");
    assert_false(figureType_ == FigureType::figureType2D, "you are plot a 3D curve on 2D plots");
    figureType_ = FigureType::figureType3D;
    fprintf(pipe_, "set colorbox\n");
    firstCurve_ = false;

    std::string size_string="";

    if(!properties.size.empty())
      size_string = "size " + properties.size;

    switch (properties.displayType) {
      case DisplayType3D::heatMap2D:fprintf(pipe_, "unset view\n");
        fprintf(pipe_, "unset pm3d\n");
        fprintf(pipe_, "set pm3d\n");
        fprintf(pipe_, "set view map\n");
        break;
      case DisplayType3D::points3D:assert_false(true, "use figure3D method instead of drawHeatMap");
        break;
      case DisplayType3D::heatMap3D:fprintf(pipe_, "unset view\n");
        fprintf(pipe_, "unset pm3d\n");
        fprintf(pipe_, "set pm3d\n");

        break;
    }

    if (!multiPlotOn_) fprintf(pipe_, "set terminal %s \"%i\" position %s %s\n", termType, figID, position_[figureNumber].c_str(), size_string.c_str());

    if (!properties.palette.empty()) fprintf(pipe_, "set palette %s\n", properties.palette.c_str());
    fprintf(pipe_, "set title \"%s\" \n", properties.title.c_str());
    fprintf(pipe_, "set xlabel \"%s\" \n", properties.xlabel.c_str());
    fprintf(pipe_, "set ylabel \"%s\" \n", properties.ylabel.c_str());
    fprintf(pipe_, "set zlabel \"%s\"  rotate left \n", properties.zlabel.c_str());
    fprintf(pipe_, "set title \"%s\" \n", properties.title.c_str());

    fprintf(pipe_, "set style fill transparent solid 0.8\n");
    fprintf(pipe_, "splot");

    std::string
        randomFileName = "/tmp/gnuplotTempData" + std::to_string(figID) + "--" + std::to_string(fileNumber_++);
    tempfiles_.push_back(randomFileName.c_str());
    tempData_ = fopen(tempfiles_.back().c_str(), "w");

    for (int k = 0; k < xCount * yCount; k++) {
      fprintf(tempData_, "%f %f %f\n", x[k], y[k], z[k]);
      if ((k + 1) % xCount == 0)
        fprintf(tempData_, "\n");
    }

    std::string title;
    if (!name.empty())
      title = "title \"" + name + "\"";
    else
      title = "notitle";

    switch (properties.displayType) {
      case DisplayType3D::heatMap2D:
        fprintf(pipe_, " \"%s\" w pm3d %s",
                tempfiles_.back().c_str(), title.c_str());
        break;
      case DisplayType3D::points3D:assert_false(true, "use figure3D method instead of drawHeatMap");
        break;
      case DisplayType3D::heatMap3D:
        fprintf(pipe_, " \"%s\" w lines lc rgb \"%s\" notitle, \"%s\" w pm3d palette title \"%s\"",
                tempfiles_.back().c_str(), properties.gridLineColor.c_str(), tempfiles_.back().c_str(), name.c_str());
        break;
    }

    fclose(tempData_);
  }

  template<typename Dtype>
  void drawPieChart(int figureNumber,
                    const std::vector<Dtype>& time,
                    const std::vector<std::string>& name,
                    std::string unit,
                    FigPropPieChart &prop) {

    Dtype A_sum = Dtype(0);
    std::vector<std::string> nameNoSpace;
    nameNoSpace.resize(name.size());
    for (int i = 0; i < name.size(); i++)
      nameNoSpace[i] = space2underscore(name[i]);
    for (int i = 0; i < name.size(); i++)
      A_sum += time[i];
    write2file(nameNoSpace, time);
    int itemNumber = name.size();

    std::string size_string="";

    if(!prop.size.empty())
      size_string = "size " + prop.size;

    fprintf(pipe_, "set terminal %s %i position %s %s\n", termType, figID, position_[figureNumber].c_str(), size_string.c_str());
    fprintf(pipe_, "set style fill solid 1.0 border rgb 'white'\n");
    fprintf(pipe_, "unset key\n");
    fprintf(pipe_, "unset tics\n");
    fprintf(pipe_, "unset border\n");
    fprintf(pipe_, "unset colorbox\n");

    fprintf(pipe_, "A_sum = %f\n", A_sum);

    fprintf(pipe_, "rowi = 0\n");
    fprintf(pipe_, "rowf = %i\n", itemNumber);
    fprintf(pipe_, "pos = 0\n");
    fprintf(pipe_, "colour = 0\n");
    fprintf(pipe_, "angle(x)=x*360/A_sum\n");
    fprintf(pipe_, "percentage(x)=x*100/A_sum\n");

    fprintf(pipe_, "centerX=-0\n");
    fprintf(pipe_, "centerY=0\n");
    fprintf(pipe_, "radius=1\n");

    fprintf(pipe_, "yposmin = -0.9*radius\n");
    fprintf(pipe_, "yposmax = 0.9*radius\n");
    fprintf(pipe_, "xpos = 1.3 * radius\n");
    fprintf(pipe_, "ypos(i) = yposmax - i*(yposmax-yposmin)/(1.0*rowf-rowi-1)\n");
    fprintf(pipe_, "set palette %s\n", prop.palette.c_str());

    fprintf(pipe_, "set size ratio -1\n");
    fprintf(pipe_, "set xrange [-radius:2.5*radius]\n");
    fprintf(pipe_, "set yrange [-radius:radius]\n");

    fprintf(pipe_, "pos = 0\n");
    fprintf(pipe_, "colour = 0\n");

    fprintf(pipe_, "filename = \"%s\"\n", tempfiles_.back().c_str());

    fprintf(pipe_, "set title \"%s (total: %.3f %s)\"\n", prop.title.c_str(), A_sum, unit.c_str());

    fprintf(pipe_,
            "plot filename u (centerX):(centerY):(radius):(pos):(pos=pos+angle($2)):((colour = colour+1)/(%i-1.0)) every ::rowi::rowf w circle palette,\
     for [i=0:rowf-rowi-1] '+' u (xpos):(ypos(i)) w p pt 5 ps 3.5 palette frac i/(%i-1.0),\
     for [i=0:rowf-rowi-1] filename u (xpos):(ypos(i)):(sprintf('%%5.2f%s - %%-40s', percentage($2), stringcolumn(1))) every ::i+rowi::i+rowi w labels left offset 3,0\n",
            itemNumber,
            itemNumber,
            "%%");
  }

  void drawPieChartWithSubCate(int figureNumber, Timer_items &items,
                               FigPropPieChart &prop) {

    std::string size_string="";

    if(!prop.size.empty())
      size_string = "size " + prop.size;

    fprintf(pipe_, "set terminal %s %i position %s %s\n", termType, figID, position_[figureNumber].c_str(), size_string.c_str());
    fprintf(pipe_, "set style fill solid 1.0 border rgb 'white'\n");

    fprintf(pipe_, "unset key\n");
    fprintf(pipe_, "unset tics\n");
    fprintf(pipe_, "unset border\n");
    fprintf(pipe_, "unset colorbox\n");
    fprintf(pipe_, "set size ratio -1\n");
    fprintf(pipe_, "set xrange [-1:2.5]\n");
    fprintf(pipe_, "set yrange [-1:1]\n");
    fprintf(pipe_, "set palette %s\n", prop.palette.c_str());

    double squareSize = std::min(3., 3. / items.totalChildren_ * 10.0);
    double fontSize = std::min(10.0, 15.0 / items.totalChildren_ * 10.0);
    std::vector<double> time;
    std::vector<std::string> name;
    double xposition = 1.05;
    double yposition = 0.9;
    double yDecrement = 1.8 / (items.totalChildren_ - 1 + 0.001);
    fprintf(pipe_, "set title \"Computation time (total: %.3f seconds)\"\n", items.time_sum_);

    int id = 1;
    printLabels(items, xposition + 0.1, yposition, yDecrement, 0.0, id, fontSize);

    yposition = 0.9;
    fprintf(pipe_, "plot");
    printLegend(items, xposition, yposition, yDecrement, 0.0, squareSize);
    printPie(items, 0, 360, 0, items.maximum_depth, 0.0, "", 100.0, 0.0);
    fprintf(pipe_, " '+' u (0):(0):(0.05) w circle fillcolor rgb 'white' lw 0.3,");
  }


 private:
  void printLabels(Timer_items &items,
                   double xposition,
                   double &yposition,
                   double yDecrement,
                   double percent,
                   int &id,
                   double fontSize) {

    if (items.id_ != -1) {
      fprintf(pipe_, "set label font \",%f\"\n", fontSize);
      fprintf(pipe_, "set label %i at %f, %f '%2.2f%%-%s' \n",
              id++, xposition, yposition, percent, items.name_.c_str());
      yposition -= yDecrement;
    }

    for (int i = 0; i < items.subItems_.size(); i++) {
      printLabels(items.subItems_[i],
                  xposition + 0.15,
                  yposition,
                  yDecrement,
                  items.subItems_[i].time_sum_ / items.time_sum_ * 100.0,
                  id,
                  fontSize);
    }
  }

  void printLegend(Timer_items &items,
                   double xposition,
                   double &yposition,
                   double yDecrement,
                   double color,
                   double squareSize) {

    if (items.id_ != -1) {
      fprintf(pipe_,
              " '+' u (%f):(%f) with point pt 5 ps %f palette frac %f,",
              xposition,
              yposition,
              squareSize,
              1.0 - color);
      yposition -= yDecrement;
    }


    for (int i = 0; i < items.subItems_.size(); i++) {
      printLegend(items.subItems_[i],
                  xposition + 0.15,
                  yposition,
                  yDecrement,
                  i / std::max((items.subItems_.size() - 1.0), 1.0),
                  squareSize);
    }
  }

  void printPie(Timer_items &items,
                double startingAngle,
                double finishingAngle,
                int depth,
                int maximumDepth,
                double color,
                std::string parent_name,
                double totalPercent, double parentPercent) {
    if (items.id_ != -1) {
      std::ostringstream hypertext;
      hypertext << items.name_<<": " << std::setprecision(4)<< totalPercent <<"%, "<< parentPercent<<"% of "<< parent_name;
      fprintf(pipe_,
              " '+' u (0):(0):(%f):(%f):(%f) w circle palette frac %f lw 0.3, '+' u (%f):(%f):(\"%s\") with labels hypertext point pt 7 ps 0 lw 0.2 lc palette frac %f,",
              1.0 - (depth - 1.0) / (maximumDepth + 1e-2) - 0.03,
              startingAngle,
              finishingAngle,
              1.0 - color,
              cos((startingAngle + finishingAngle) / 2.0 / 180 * M_PI) * (1.0 - (depth - 0.5) / (maximumDepth + 1e-2)),
              sin((startingAngle + finishingAngle) / 2.0 / 180 * M_PI) * (1.0 - (depth - 0.5) / (maximumDepth + 1e-2)),
              hypertext.str().c_str(),
              1.0 - color);

      fprintf(pipe_, " '+' u (0):(0):(%f):(%f):(%f) w circle fillcolor rgb 'white' lw 0.3,",
                1.0 - (depth + 0.0) / (maximumDepth + 1e-2), startingAngle, finishingAngle);

    }

    double totalAngle = finishingAngle - startingAngle;
    double angle = startingAngle;

    for (int i = 0; i < items.subItems_.size(); i++) {
      printPie(items.subItems_[i],
               angle,
               angle + totalAngle * items.subItems_[i].time_sum_ / items.time_sum_,
               depth + 1,
               maximumDepth,
               i / std::max((items.subItems_.size() - 1.0), 1.0),
               items.name_,
               totalPercent * items.subItems_[i].time_sum_ / items.time_sum_,
               items.subItems_[i].time_sum_ / items.time_sum_ * 100.0);

      angle += totalAngle * items.subItems_[i].time_sum_ / items.time_sum_;
    }
  }

 public:
  /* Drawing only to the screen */
  void drawFigure() {
    fprintf(pipe_, "\n");
    fflush(pipe_);
    figureType_ = FigureType::No_figure;
  }

  /* Output to the sceent and also to a file */
  void drawFigure(OutputFormat format, bool saveAllFigures, std::string& logPath) {
    fprintf(pipe_, "\n");
    export2File(format, saveAllFigures, logPath);
    figureType_ = FigureType::No_figure;
  }

  void export2File(OutputFormat format, bool saveAllFigures, std::string &path) {
    fprintf(pipe_, "set terminal %s size 10.5, 6 enhanced font 'Times Roman,20'\n", OutputFormatStringMap_[format].c_str());
    std::ostringstream figPath;
    if (!saveAllFigures) timeID_ = 0;
    figPath << path << "/Figure" << figID << "-" << timeID_++
            << "." << OutputFormatStringMap_[format].c_str();
    fprintf(pipe_, "set output '%s'\n", figPath.str().c_str());
    fprintf(pipe_, "replot\n");
    fprintf(pipe_, "unset output\n");
    fflush(pipe_);
  }

  /* adding a curve to a graph */
  template<typename Dtype>
  void appendData(Dtype *x, Dtype *y, int dataLen, std::string name) {
    assert_false(figureType_ == FigureType::figureType3D, "you are plotting a 2D curve on 3D plots");
    assert_false(figureType_ == FigureType::No_figure, "call figure() or figure3D() before plot");
    write2file(x, y, dataLen);
    if (!firstCurve_) fprintf(pipe_, ",");
    firstCurve_ = false;

    std::string title;
    if (!name.empty())
      title = "title \"" + name + "\"";
    else
      title = "notitle";

    fprintf(pipe_, " \"%s\" with lines %s ", tempfiles_.back().c_str(), title.c_str());
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
  void appendData(Dtype *x,
                  Dtype *y,
                  int dataLen,
                  PlotMethods2D plottingMethod,
                  std::string name,
                  std::string style) {
//    assert_false(currentFigure_ == figureType3D, "you are plot a 2D curve on 3D plots");
    assert_false(figureType_ == FigureType::No_figure, "call figure() or figure3D() before plot");
    write2file(x, y, dataLen);
    if (!firstCurve_) fprintf(pipe_, ",");

    std::string title;
    if (!name.empty())
      title = "title \"" + name + "\"";
    else
      title = "notitle";

    firstCurve_ = false;
    fprintf(pipe_,
            " \"%s\" with %s %s %s ",
            tempfiles_.back().c_str(),
            plottingMethodsEnumToStringMap_[plottingMethod].c_str(),
            style.c_str(),
            title.c_str());
  }

  template<typename Dtype>
  void appendArrows(Dtype *startingX,
                    Dtype *startingY,
                    Dtype *startingZ,
                    Dtype *arrowX,
                    Dtype *arrowY,
                    Dtype *arrowZ,
                    int dataLen,
                    std::string name,
                    std::string style) {
    assert_false(figureType_ == FigureType::No_figure, "call figure() or figure3D() before plot");
    write2file(startingX, startingY, startingZ, arrowX, arrowY, arrowZ, dataLen);
    if (!firstCurve_) fprintf(pipe_, ",");
    firstCurve_ = false;
    fprintf(pipe_,
            " \"%s\" using 1:2:3:4:5:6 with %s %s title \"%s\" ",
            tempfiles_.back().c_str(),
            "vectors",
            style.c_str(),
            name.c_str());
  }

  template<typename Dtype>
  void appendArrows(Dtype *startingX,
                    Dtype *startingY,
                    Dtype *arrowX,
                    Dtype *arrowY,
                    int dataLen,
                    std::string name,
                    std::string style) {
    assert_false(figureType_ == FigureType::No_figure, "call figure() or figure3D() before plot");
    write2file(startingX, startingY, arrowX, arrowY, dataLen);
    if (!firstCurve_) fprintf(pipe_, ",");
    firstCurve_ = false;
    fprintf(pipe_,
            " \"%s\" using 1:2:3:4 with %s %s title \"%s\" ",
            tempfiles_.back().c_str(),
            "vectors",
            style.c_str(),
            name.c_str());
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
  void appendDataAndStd(Dtype *x, Dtype *y, Dtype *std, int dataLen, std::string name, std::string style) {
    assert_false(figureType_ == FigureType::figureType3D, "you are plot a 2D curve on 3D plots");
    assert_false(figureType_ == FigureType::No_figure, "call figure() or figure3D() before appending Data");
    write2fileWithStd(x, y, std, dataLen);
    if (!firstCurve_) fprintf(pipe_, ",");
    firstCurve_ = false;
    fprintf(pipe_,
            " \"%s\" u 1:2 with lines %s title \"%s\", ",
            tempfiles_.back().c_str(),
            style.c_str(),
            name.c_str());
    fprintf(pipe_, " \"%s\" u 1:3:4 with filledcu %s notitle", tempfiles_.back().c_str(), style.c_str());
  }

  /*
  heatOn: value of z is also represented by color
 */
  template<typename Dtype>
  void append3D_Data(Dtype *x,
                     Dtype *y,
                     Dtype *z,
                     int dataLen,
                     bool heatOn,
                     PlotMethods3D method,
                     std::string name,
                     std::string style) {
    assert_false(figureType_ == FigureType::figureType2D, "you are plot a 3D curve on 2D plots");
    assert_false(figureType_ == FigureType::No_figure, "call figure() or figure3D() before appending Data");

    write2file(x, y, z, dataLen);
    if (!firstCurve_) fprintf(pipe_, ",");
    firstCurve_ = false;
    std::string palette;
    if (heatOn) palette = "palette";
    else palette = "";
    fprintf(pipe_, " \"%s\" using 1:2:3 w %s %s %s title \"%s\"", tempfiles_.back().c_str(),
            plottingMethods3DEnumToStringMap_[method].c_str(),
            palette.c_str(),
            style.c_str(),
            name.c_str());
  }

 private:

  void assert_false(bool condition, std::string message) {
    if (condition) {
      std::cout << "Gnuplot_cpp: " << message << std::endl;
      throw 0;
    }
  }

  template<typename Dtype>
  void write2file(Dtype *x, Dtype *y, int dataLength) {
    std::string
        randomFileName = "/tmp/gnuplotTempData" + std::to_string(figID) + "--" + std::to_string(fileNumber_++);
    tempfiles_.push_back(randomFileName);
    tempData_ = fopen(tempfiles_.back().c_str(), "wt");
    for (int k = 0; k < dataLength; k++)
      fprintf(tempData_, "%f %f\n", x[k], y[k]);
    fclose(tempData_);
  }

  template<typename Dtype>
  void write2file(Dtype *x, Dtype *y, Dtype *z, int dataLength) {
    std::string
        randomFileName = "/tmp/gnuplotTempData" + std::to_string(figID) + "--" + std::to_string(fileNumber_++);
    tempfiles_.push_back(randomFileName);
    tempData_ = fopen(tempfiles_.back().c_str(), "w");
    for (int k = 0; k < dataLength; k++)
      fprintf(tempData_, "%f %f %f\n", x[k], y[k], z[k]);
    fclose(tempData_);
  }

  template<typename Dtype>
  void write2file(Dtype *x, Dtype *y, Dtype *z, Dtype *w, int dataLength) {
    std::string
        randomFileName = "/tmp/gnuplotTempData" + std::to_string(figID) + "--" + std::to_string(fileNumber_++);
    tempfiles_.push_back(randomFileName);
    tempData_ = fopen(tempfiles_.back().c_str(), "w");
    for (int k = 0; k < dataLength; k++)
      fprintf(tempData_, "%f %f %f %f\n", x[k], y[k], z[k], w[k]);
    fclose(tempData_);
  }

  template<typename Dtype>
  void write2file(Dtype *x1, Dtype *y1, Dtype *z1, Dtype *x2, Dtype *y2, Dtype *z2, int dataLength) {
    std::string
        randomFileName = "/tmp/gnuplotTempData" + std::to_string(figID) + "--" + std::to_string(fileNumber_++);
    tempfiles_.push_back(randomFileName);
    tempData_ = fopen(tempfiles_.back().c_str(), "w");
    for (int k = 0; k < dataLength; k++)
      fprintf(tempData_, "%f %f %f %f %f %f\n", x1[k], y1[k], z1[k], x2[k], y2[k], z2[k]);
    fclose(tempData_);
  }

  template<typename Dtype>
  void write2fileWithStd(Dtype *x, Dtype *y, Dtype *std, int dataLength) {
    std::string
        randomFileName = "/tmp/gnuplotTempData" + std::to_string(figID) + "--" + std::to_string(fileNumber_++);
    tempfiles_.push_back(randomFileName);
    tempData_ = fopen(tempfiles_.back().c_str(), "w");
    for (int k = 0; k < dataLength; k++)
      fprintf(tempData_, " %f %f %f %f\n", x[k], y[k], y[k] - std[k], y[k] + std[k]);
    fclose(tempData_);
  }
  template<typename Dtype>
  void write2file(std::vector<std::string> name,
                  std::vector<Dtype> time) {
    std::string
        randomFileName = "/tmp/gnuplotTempData" + std::to_string(figID) + "--" + std::to_string(fileNumber_++);
    tempfiles_.push_back(randomFileName.c_str());
    tempData_ = fopen(tempfiles_.back().c_str(), "w");
    for (int k = 0; k < time.size(); k++) {
      fprintf(tempData_, "%s %f", name[k].c_str(), time[k]);
      if (k != time.size() - 1)
        fprintf(tempData_, "\n");
    }
    fclose(tempData_);
  }

  std::string space2underscore(std::string text) {
    for (std::string::iterator it = text.begin(); it != text.end(); ++it) {
      if (*it == ' ') {
        *it = '_';
      }
    }
    return text;
  }

  FILE *pipe_, *tempData_;
  std::vector<std::string> tempfiles_;
  FigureType figureType_ = FigureType::No_figure;
  int figID = 0;
  int fileNumber_ = 0;
  bool firstCurve_ = true;
  std::map<PlotMethods2D, std::string> plottingMethodsEnumToStringMap_;
  std::map<PlotMethods3D, std::string> plottingMethods3DEnumToStringMap_;
  std::map<OutputFormat, std::string> OutputFormatStringMap_;
  std::vector<std::string> position_ = {"0,0", "685,0", "1300,0", "0,645", "685,645", "1300,645"};
  int timeID_ = 0;

  bool multiPlotOn_ = false;

};

}
}
}

#endif //RAI_GNUPLOT_CPP_HPP
