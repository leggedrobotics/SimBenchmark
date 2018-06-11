#ifndef RAI_GOLDENSECTIONMETHOD_HPP
#define RAI_GOLDENSECTIONMETHOD_HPP

#include <float.h>                          // required for DBL_EPSILON
#include <math.h>                           // required for fabs(), sqrt()
#include <functional>


////////////////////////////////////////////////////////////////////////////////
//  void Min_Search_Golden_Section(double (*f)(double), double* a, double *fa,//
//                                  double* b, double* fb, double tolerance)  //
//                                                                            //
//  Description:                                                              //
//     This routine can be used to locate the minimum of a unimodal function  //
//     within a preassigned tolerance.                                        //
//                                                                            //
//     The Golden Section search is similar to the Fibonacci search only the  //
//     internal points of the interval are chosen so that the distance of the //
//     right-most internal point to the left-most endpoint is equal to the    //
//     distance of the right-most endpoint to the left-most internal point.   //
//     Both the distances are equal to a fixed constant, lambda, times the    //
//     length of the interval.  In order that lambda be fixed for each        //
//     iteration it is easy to show that lambda^2 + lambda - 1 = 0 or         //
//     lambda = (sqrt(5) - 1) / 2.  The name Golden Section comes from Euclid,//
//     and is equivalent to dividing a unit interval into two sections such   //
//     that the ratio of the whole to the larger part equals the ratio of the //
//     larger part to the smaller part.  The larger part is lambda.           //
//                                                                            //
//     The algorithm proceeds as follows: Given an initial interval [a,b],    //
//     calculate the internal points x1 = b - lambda * (b - a) and            //
//     x2 = a + lambda (b - a).  Iterate unless the stopping criterion is met.//
//     Otherwise if ( f(x1) < f(x2) ) then the interval (x2,b] can be         //
//     deleted, so set the new value for b = x2, set x2 = x1, and calculate   //
//     the new value for x1 = a + (1 - lambda) * (b - a).  Otherwise the      //
//     interval [a,x1) can be eliminated, so set the new value for a = x1,    //
//     set x1 = x2, and set new value for x2 = b - (1 - lambda) * (b - a).    //
//                                                                            //
//  Arguments:                                                                //
//     double (*f)(double)                                                    //
//        Pointer to a user-defined function of a real variable (type double) //
//        returning a real number (type double).                              //
//     double *a                                                              //
//        On input, *a is the lower endpoint of the initial interval which    //
//        contains a local minimum.  On output, *a is the lower endpoint of   //
//        the final subinterval which contains the local minimum.             //
//     double *fa                                                             //
//        On input and output, *fa is the value of f(x) evaluated at *a.      //
//     double *b                                                              //
//        On input, *b is the upper endpoint of the initial interval which    //
//        contains a local minimum.  On output, *b is the upper endpoint of   //
//        the final subinterval which contains the local minimum.             //
//     double *fb                                                             //
//        On input and output, *fb is the value of f(x) evaluated at *b.      //
//     double tolerance                                                       //
//        A parameter which controls the termination of the procedure.        //
//        If the magnitude of the midpoint of the interval of uncertainty is  //
//        less than or equal to 1, then the procedure is terminated if the    //
//        length of the interval of uncertainty is less than the "tolerance". //
//        If the magnitude of the midpoint of the interval of uncertainty is  //
//        greater than 1, then the procedure is terminated when the length    //
//        of the interval of uncertainty relative to the magnitude of the     //
//        midpoint of the interval of uncertainty is smaller than "tolerance".//
//        If a nonpositive number is passed, then "tolerance" is set to       //
//        sqrt(DBL_EPSILON) * fabs(*b - *a).                                  //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     extern double f(double);                                               //
//     double a = -1.0;                                                       //
//     double b =  1.0;                                                       //
//     double fa = f(a);                                                      //
//     double fb = f(b);                                                      //
//     double tolerance = 1.0e-2;                                             //
//                                                                            //
//     Min_Search_Golden_Section( f, &a, &fa, &b, &fb, tolerance);            //
//     printf("local minimum is between (%12.6e,%12.6e) and (%12.6e,%12.6e)\n"//
//                                                              , a,fa, b,fb);//
////////////////////////////////////////////////////////////////////////////////

namespace rai {
namespace Algorithm {

class GoldenSectionMethod {

  using Fcn = std::function<double(double)>;

 public:

  constexpr static double sqrt5 = 2.236067977499789696;

  inline static void solve(Fcn f, double *a, double *fa,
                           double *b, double *fb, double tolerance) {
    static const double lambda = 0.5 * (sqrt5 - 1.0);
    static const double mu = 0.5 * (3.0 - sqrt5);         // = 1 - lambda
    double x1;
    double x2;
    double fx1;
    double fx2;


    // Find first two internal points and evaluate
    // the function at the two internal points.

    x1 = *b - lambda * (*b - *a);
    x2 = *a + lambda * (*b - *a);
    fx1 = f(x1);
    fx2 = f(x2);

    // Loop by exluding segments from current endpoints a, b
    // to current internal points x1, x2 and then calculating
    // a new internal point until the length of the interval
    // is less than or equal to the tolerance.//

    while (!Stopping_Rule(*a, *b, tolerance)) {
      if (fx1 > fx2) {
        *a = x1;
        *fa = fx1;
        if (Stopping_Rule(*a, *b, tolerance)) break;
        x1 = x2;
        fx1 = fx2;
        x2 = *b - mu * (*b - *a);
        fx2 = f(x2);
      } else {
        *b = x2;
        *fb = fx2;
        if (Stopping_Rule(*a, *b, tolerance)) break;
        x2 = x1;
        fx2 = fx1;
        x1 = *a + mu * (*b - *a);
        fx1 = f(x1);
      }
    }
    return;
  }

 private:

////////////////////////////////////////////////////////////////////////////////
//  static int Stopping_Rule(double x0, double x1, double tolerance)          //
//                                                                            //
//  Description:                                                              //
//     Returns true (1) if |x1 - x0| < tolerance * | (x1 + x0) / 2 | otherwise//
//     returns false (0) provided that | (x1 + x0) / 2 | > 1.                 //
//     If | (x1 + x0) / 2 | <= 1, then the function returns true (1) if       //
//     |x1 - x0| < tolerance, otherwise returns false (0).                    //
//                                                                            //
//  Note! For compilers which allow the inline modifier this routine should   //
//  be declared with the inline modifier.                                     //
//                                                                            //
//  Arguments:                                                                //
//     double x0                                                              //
//        Either the previous estimate or the current estimate of the relative//
//        extremum.                                                           //
//     double x1                                                              //
//        If x0 is the previous estimate of the relative extremum then x1 is  //
//        the current estimate of the relative extremum and if x0 is the      //
//        the current estimate of the relative extremum then x1 is the        //
//        previous estimate of the relative extremum.                         //
//     double tolerance                                                       //
//        A parameter which controls the termination of the procedure.        //
//        If the magnitude of the midpoint of the interval of uncertainty is  //
//        less than or equal to 1, then the procedure is terminated if the    //
//        length of the interval of uncertainty is less than the "tolerance". //
//        If the magnitude of the midpoint of the interval of uncertainty is  //
//        greater than 1, then the procedure is terminated when the length    //
//        of the interval of uncertainty relative to the magnitude of the     //
//        midpoint of the interval of uncertainty is smaller than "tolerance".//
//        "tolerance" should be a strictly positive number.                   //
////////////////////////////////////////////////////////////////////////////////

//  inline static int Stopping_Rule(double x0, double x1, double tolerance) {
//    double xm = 0.5 * std::fabs(x1 + x0);
//
//    if (xm <= 1.0) return (std::fabs(x1 - x0) < tolerance) ? 1 : 0;
//    return (std::fabs(x1 - x0) < tolerance * xm) ? 1 : 0;
//  }

  inline static bool Stopping_Rule(double x0, double x1, double tolerance) {
    return std::fabs(x1 - x0) < tolerance;
  }

};
}
}

#endif //RAI_GOLDENSECTIONMETHOD_HPP
