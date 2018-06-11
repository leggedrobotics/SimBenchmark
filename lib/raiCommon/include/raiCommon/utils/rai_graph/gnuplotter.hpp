//
// Created by jemin on 26.07.16.
//

/*
 * install gnuplot by "sudo apt-get install gnuplot-x11" *
 */

#ifndef GNUPLOTWRAPPER_GNUPLOTTER_HPP
#define GNUPLOTWRAPPER_GNUPLOTTER_HPP

#include "gnuplotFigureManager.hpp"

namespace rai {
namespace Utils {
extern std::shared_ptr<Graph::GnuplotFigureManager> graph;

}
}

#endif //GNUPLOTWRAPPER_GNUPLOTTER_HPP
