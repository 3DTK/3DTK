#ifndef __ACCUMULATOR__
#define __ACCUMULATOR__
#include <set>
#include "shapes/ConfigFileHough.h"
#include "slam6d/point.h"
using std::multiset;
#include "shapes/hsm3d.h"


double* polar2normal(double theta, double phi);

/**
  * Accumulator for the Hough Transform. For a detailed explanation of the
  * different accumulator types please see:
  * Dorit Borrmann, Jan Elseberg, Kai Lingemann, and Andreas Nüchter. 
  * A Data Structure for the 3D Hough Transform for Plane Detection. 
  * In Proceedings of the 7th IFAC Symposium on Intelligent Autonomous Vehicles (IAV '10), 
  * Lecce, Italy, September 2010. 
  */
class Accumulator {
  
  public:
    /** Contains the configuration for the accumulator */
    ConfigFileHough myConfigFileHough;
    /** TODO */
    int count;
    /** Constructor */
    Accumulator() { }
    /** Destructor */
    virtual ~Accumulator() { }
    /** Prints the accumulator so that the data can be shown using gnuplot */
    virtual void printAccumulator() = 0;
    /** Sets the counters for each accumulator cells back to 0 */
    virtual void resetAccumulator() = 0;
    /** Accumulates the cell containing theta, phi and rho.
     * A plane is represented by:
     * rho = cos(theta)*sin(phi)*x + sin(phi)*sin(theta)*y + cos(phi)*z
     * @param theta theta angle of the plane
     * @param phi phi angle of the plane
     * @param rho distance of the plane
     */
    virtual bool accumulate(double theta, double phi, double rho) = 0;
    /** Accumulate all the cells that correspond to planes that go through p.
     * A plane is represented by:
     * rho = cos(theta)*sin(phi)*x + sin(phi)*sin(theta)*y + cos(phi)*z
     * @param p the point that is transformed into Hough Space
     */
    virtual void accumulate(Point p) = 0;
    /** Accumulates all the cells that correspond to planes that go through p.
     * @param p the point that is transformed into Hough Space
     * @return the plane whose counter has exceeded the 
     * ConfigFileHough.GetAccumulatorMax , or {-1,_,_}
     */
    virtual double* accumulateRet(Point p) = 0;
    /** Accumulate all the cells that correspond to planes that go through p.
     * @param p the point that is transformed into Hough Space
     * @return the cell that has the maximum counter of all cells touched by the
     * Hough Transform of p
     */
    virtual int* accumulateAPHT(Point p) = 0;
    /** 
     * Given the representation (rho, theta, phi) of a plane, the function
     * calculates the center of the cell that this plane belongs to.
     * A plane is represented by:
     * rho = cos(theta)*sin(phi)*x + sin(phi)*sin(theta)*y + cos(phi)*z
     * @param theta theta angle of the plane
     * @param phi phi angle of the plane
     * @param rho distance of the plane (newly calculated distance will be
     * written here)
     * @return the normal vector of the plane in the center of the cell
     */
    virtual double* getMax(double &rho, double &theta, double &phi) = 0;
    /**
     * Given a cell in the accumulator, the function calculates the plane that
     * is represented by this cell, i.e., the rho, theta, phi representation of
     * the plane in the center of the cell.
     * @param cell the indices of the cell in the accumulator
     * @return normal vector (x,y,z) and distance (rho) of the plane representation {x, y, z, rho}
     */
    virtual double* getMax(int* cell) = 0;
    /**
     * Returns a sorted list of the all cells in the accumulator.
     * @return a sorted multiset containing the cells as (counter, rho_index, theta_index, phi_index)
     */
    virtual multiset<int*, maxcompare>* getMax() = 0; 
    /**
     * Cleans the accumulator using a very simple sliding window strategy. A
     * quadratic window is moved over the accumulator. In each step all the
     * counters in the window except the maximum counter are set to 0.
     * @param the size of the window
     */
    virtual void peakWindow(int size) = 0;
};

/**
 * The AccumulatorSimple represents the Hough Space as an array. Each dimension
 * is evenly discretized. This means that when projected onto the unit sphere
 * the cells vary significantly in size.
 */
class AccumulatorSimple : public Accumulator {
  public:
    AccumulatorSimple(ConfigFileHough myCfg);
    virtual ~AccumulatorSimple();
    virtual void printAccumulator();
    void resetAccumulator();
    bool accumulate(double theta, double phi, double rho);
    void accumulate(Point p);
    double* accumulateRet(Point p);
    int* accumulateAPHT(Point p);
    double* getMax(double &rho, double &theta, double &phi);
    double* getMax(int* cell);
    void peakWindow(int size);
    multiset<int*, maxcompare>* getMax(); 
  private:
    int ***accumulator;
};

/**
  * The AccumulatorCube maps the unit sphere onto a cube. Each face of the cube
  * is evenly discretized.
  */
class AccumulatorCube : public Accumulator {
  public:
    AccumulatorCube(ConfigFileHough myCfg);
    virtual ~AccumulatorCube();
    virtual void printAccumulator();
    void printAccumulator2();
    void resetAccumulator();
    void peakWindow(int size);
    bool accumulate(double theta, double phi, double rho);
    void accumulate(Point p);
    double* accumulateRet(Point p);
    int* accumulateAPHT(Point p);
    double* getMax(double &rho, double &theta, double &phi);
    double* getMax(int* cell);
    multiset<int*, maxcompare>* getMax(); 
  private:
    int nrCells;
    int ****accumulator;
    buffer_point coords_s2_to_cell(double *n, unsigned int width);
    double* coords_cube_to_s2(buffer_point lastbp, unsigned int width);
    void coords_cube_for_print(buffer_point src, double** result, unsigned int width);
    buffer_point lastbp;
};

/**
  * The AccumulatorBall discretizes the unit sphere slice-wise. For each slice a
  * a stepwidth in calculated for discretization in direction of theta is
  * calculated.
  */
class AccumulatorBall : public Accumulator {
  public:
    AccumulatorBall(ConfigFileHough myCfg);
    virtual ~AccumulatorBall();
    virtual void printAccumulator();
    void resetAccumulator();
    bool accumulate(double theta, double phi, double rho);
    void accumulate(Point p);
    double* accumulateRet(Point p);
    int* accumulateAPHT(Point p);
    double* getMax(double &rho, double &theta, double &phi);
    double* getMax(int* cell);
    multiset<int*, maxcompare>* getMax(); 
    void peakWindow(int size);
  private:
    int ***accumulator;
    int *ballNr;
};

/**
  * The AccumulatorBallI is an improvement of the AccumulatorBall. It
  * discretizes the unit sphere slice-wise. For each slice a stepwidth in
  * calculated for discretization in direction of theta is calculated. The
  * difference to AccumulatorBall is that both of the poles are covered by on
  * cell each.
  */
class AccumulatorBallI : public Accumulator {
  public:
    AccumulatorBallI(ConfigFileHough myCfg);
    virtual ~AccumulatorBallI();
    virtual void printAccumulator();
    void resetAccumulator();
    bool accumulate(double theta, double phi, double rho);
    void accumulate(Point p);
    double* accumulateRet(Point p);
    int* accumulateAPHT(Point p);
    double* getMax(double &rho, double &theta, double &phi);
    double* getMax(int* cell);
    multiset<int*, maxcompare>* getMax(); 
    void peakWindow(int size);
  private:
    int ***accumulator;
    int *ballNr;
    double step; // in degree
    double phi_top_deg;
    double phi_top_rad;
};

#endif
