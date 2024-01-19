#include <array>
#include <iostream>
#include "cuddObj.hh"
#include "SymbolicSet.hh"
#include "SymbolicModelGrowthBound.hh"
#include "TicToc.hh"
#include "RungeKutta4.hh"
#include "FixedPoint.hh"

/* state space dim */
#define sDIM 3
#define iDIM 2

/* data types for the ode solver */
typedef std::array<double, 3> state_type;
typedef std::array<double, 2> input_type;

/* sampling time */
const double tau = 0.2;
/* number of intermediate steps in the ode solver */
const int nint = 5;
OdeSolver ode_solver(sDIM, nint, tau);

/* the vehicle ode   */
auto  vehicle_post = [](state_type &x, input_type &u) -> void {
  auto rhs = [](state_type& xx, const state_type &x, input_type &u) {
      double alpha=std::atan(std::tan(u[1])/2.0);
      xx[0] = u[0]*std::cos(alpha+x[2])/std::cos(alpha);
      xx[1] = u[0]*std::sin(alpha+x[2])/std::cos(alpha);
      xx[2] = u[0]*std::tan(u[1]);
  };
  ode_solver(rhs, x, u);
};

scots::SymbolicSet vehicleCreateStateSpace(Cudd &mgr);
scots::SymbolicSet vehicleCreateInputSpace(Cudd &mgr);
void vehicleCreateObstacles(scots::SymbolicSet &obs);

/****************************************************************************/

int main() {

  char filename1[256], filename2[256], filenameA[256], filenameB[256];
  Cudd mgr;
  scots::SymbolicSet is=vehicleCreateInputSpace(mgr);

  /* computation of the growth bound (the result is stored in r)  */
  auto radius_post = [](state_type &r, input_type &u) -> void {
    double c = std::abs(u[0]*std::sqrt(std::tan(u[1])*std::tan(u[1])/4.0+1));
    r[0] = r[0]+c*r[2]*0.2;
    r[1] = r[1]+c*r[2]*0.2;
  };

  TicToc tt;

/* Round 1 *************************************************************************************************/

  /* construct SymbolicSet for the state space */
  double lb1[sDIM]={0.6, 0, -M_PI};  
  double ub1[sDIM]={5.4, 4.6, M_PI}; 
  double eta1[sDIM]={0.16, 0.16, 0.16};   
  scots::SymbolicSet ss1(mgr, sDIM, lb1, ub1, eta1);
  ss1.addGridPoints();

  double H11[2*sDIM]={3, -4, 0,
                      -3, -4, 0};
  double h11[2] = {-9.4, -27.4};
  ss1.remPolytope(2, H11, h11, scots::OUTER);

  sprintf(filenameA, "%s%d.bdd", "AbstractSet", 1);
  ss1.writeToFile(filenameA);

  /* construct the obstacles */
  scots::SymbolicSet obs1(ss1);
  vehicleCreateObstacles(obs1);
  sprintf(filenameB, "%s%d.bdd", "Obstacle", 1);
  obs1.writeToFile(filenameB);

  /* setup class for symbolic model computation */
  scots::SymbolicSet sspost1(ss1, 1);
  scots::SymbolicModelGrowthBound<state_type,input_type> abstraction1(&ss1, &is, &sspost1);
  std::cout << std::endl << "Interation 1" << std::endl;
  tt.tic();
  abstraction1.computeTransitionRelation(vehicle_post, radius_post);
  std::cout << std::endl;
  tt.toc();
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction1.getSize() << std::endl;
  std::cout << std::endl;

  /* controller synthesis */
  scots::SymbolicSet ts1(ss1);
  double H12[4*sDIM]={-1,  0, 0,
                       1,  0, 0,
                       0, -1, 0,
                       0,  1, 0};
  double h12[4] = {-3, 3.5, -3, 3.5};
  ts1.addPolytope(4, H12, h12, scots::INNER);
  sprintf(filename1, "%s%d.bdd", "Target", 1);
  ts1.writeToFile(filename1);

  scots::FixedPoint fp1(&abstraction1);
  BDD T1 = ts1.getSymbolicSet();
  BDD O1 = obs1.getSymbolicSet();
  tt.tic();
  BDD C1=fp1.reachAvoid(T1, O1, 1);
  tt.toc();

  scots::SymbolicSet controller1(ss1, is);
  controller1.setSymbolicSet(C1);
  sprintf(filename2, "%s%d.bdd", "Controller", 1);
  controller1.writeToFile(filename2);
  std::cout << std::endl;

/* Round 2 *************************************************************************************************/

  /* construct SymbolicSet for the state space */
  double lb2[sDIM]={0.6, 0.4, -M_PI};  
  double ub2[sDIM]={5.4, 7.6, M_PI}; 
  double eta2[sDIM]={0.16, 0.16, 0.16};   
  scots::SymbolicSet ss2(mgr, sDIM, lb2, ub2, eta2);
  ss2.addGridPoints();

  double H21[4*sDIM]={3, -4, 0,
                     -3, -4, 0,
                      3,  4, 0, 
                     -3,  4, 0};
  double h21[4] = {-21.4, -39.4, 10.6, -7.4};
  ss2.remPolytope(4, H21, h21, scots::OUTER);

  sprintf(filenameA, "%s%d.bdd", "AbstractSet", 2);
  ss2.writeToFile(filenameA);

  /* construct the obstacles */
  scots::SymbolicSet obs2(ss2);
  vehicleCreateObstacles(obs2);
  sprintf(filenameB, "%s%d.bdd", "Obstacle", 2);
  obs2.writeToFile(filenameB);

  /* setup class for symbolic model computation */
  scots::SymbolicSet sspost2(ss2, 1);
  scots::SymbolicModelGrowthBound<state_type,input_type> abstraction2(&ss2, &is, &sspost2);
  std::cout << std::endl << "Interation 2" << std::endl;
  tt.tic();
  abstraction2.computeTransitionRelation(vehicle_post, radius_post);
  std::cout << std::endl;
  tt.toc();
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction2.getSize() << std::endl;
  std::cout << std::endl;

  /* controller synthesis */
  scots::SymbolicSet ts2(ss2);
  double H22[4*sDIM]={-1,  0, 0,
                       1,  0, 0,
                       0, -1, 0,
                       0,  1, 0};
  double h22[4] = {-3, 3.5, -6.25, 6.75};
  ts2.addPolytope(4, H22, h22, scots::INNER);
  sprintf(filename1, "%s%d.bdd", "Target", 2);
  ts2.writeToFile(filename1);

  scots::FixedPoint fp2(&abstraction2);
  BDD T2 = ts2.getSymbolicSet();
  BDD O2 = obs2.getSymbolicSet();
  tt.tic();
  BDD C2=fp2.reachAvoid(T2, O2, 1);
  tt.toc();

  scots::SymbolicSet controller2(ss2, is);
  controller2.setSymbolicSet(C2);
  sprintf(filename2, "%s%d.bdd", "Controller", 2);
  controller2.writeToFile(filename2);
  std::cout << std::endl;

/* Round 3 ********************************************************************************************/

  /* SymbolicSet for the state space */
  double lb3[sDIM]={0.6, 5.4, -M_PI};  
  double ub3[sDIM]={5.4, 10, M_PI}; 
  double eta3[sDIM]={0.18, 0.18, 0.18};   
  scots::SymbolicSet ss3(mgr, sDIM, lb3, ub3, eta3);
  ss3.addGridPoints();
 
  double H31[2*sDIM]={-3, 4, 0,
                      3,  4, 0};
  double h31[2] = {12.6, 30.6};
  ss3.remPolytope(2, H31, h31, scots::OUTER);

  sprintf(filenameA,"%s%d.bdd","AbstractSet", 3);
  ss3.writeToFile(filenameA);

  /* construct the obstacles */
  scots::SymbolicSet obs3(ss3);
  vehicleCreateObstacles(obs3);
  sprintf(filenameB,"%s%d.bdd", "Obstacle", 3);
  obs3.writeToFile(filenameB);

  /* Transition Relation */
  scots::SymbolicSet sspost3(ss3, 1);
  scots::SymbolicModelGrowthBound<state_type,input_type> abstraction3(&ss3, &is, &sspost3);
  std::cout << std::endl << "Interation 3" << std::endl;
  tt.tic();
  abstraction3.computeTransitionRelation(vehicle_post, radius_post);
  std::cout << std::endl;
  tt.toc();
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction3.getSize() << std::endl;
  std::cout << std::endl;

  /* Controller Synthesis 3-1 */
  scots::SymbolicSet ts31(ss3);
  double H32[4*sDIM]={-1,  0, 0,
                        1,  0, 0,
                        0, -1, 0,
                        0,  1, 0};
  double h32[4] = {-1.5, 2, -9, 9.5};
  ts31.addPolytope(4, H32, h32, scots::INNER);
  sprintf(filename1, "%s%d.bdd", "Target", 3);
  ts31.writeToFile(filename1);
  scots::FixedPoint fp31(&abstraction3);
  BDD T31 = ts31.getSymbolicSet();
  BDD O3 = obs3.getSymbolicSet();
  tt.tic();
  BDD C31=fp31.reachAvoid(T31, O3, 1);
  tt.toc();

  scots::SymbolicSet controller31(ss3, is);
  controller31.setSymbolicSet(C31);
  sprintf(filename2, "%s%d.bdd", "Controller", 3);
  controller31.writeToFile(filename2);
  std::cout << std::endl; 

  /* Controller Synthesis 3-2 */
  scots::SymbolicSet ts32(ss3);
  double H33[4*sDIM]={-1,  0, 0,
                        1,  0, 0,
                        0, -1, 0,
                        0,  1, 0};
  double h33[4] = {-4.75, 5.25, -9, 9.5};
  ts32.addPolytope(4, H33, h33, scots::INNER);
  sprintf(filename1, "%s%d.bdd", "Target", 4);
  ts32.writeToFile(filename1);

  scots::FixedPoint fp32(&abstraction3);
  BDD T32 = ts32.getSymbolicSet();
  tt.tic();
  BDD C32=fp32.reachAvoid(T32, O3, 1);
  tt.toc();

  scots::SymbolicSet controller32(ss3, is);
  controller32.setSymbolicSet(C32);
  sprintf(filename2, "%s%d.bdd", "Controller", 4);
  controller32.writeToFile(filename2);
  std::cout << std::endl;

/* Round 4 *************************************************************************************************/

  /* construct SymbolicSet for the state space */
  double lb4[sDIM]={4.6, 5.4, -M_PI};  
  double ub4[sDIM]={9.4, 10, M_PI}; 
  double eta4[sDIM]={0.16, 0.16, 0.16};   
  scots::SymbolicSet ss4(mgr, sDIM, lb4, ub4, eta4);
  ss4.addGridPoints();

  double H41[2*sDIM]={-3,  4, 0,
                      3,  4, 0};
  double h41[2] = {0.6, 42.6};
  ss4.remPolytope(4, H41, h41, scots::OUTER);

  sprintf(filenameA, "%s%d.bdd", "AbstractSet", 4);
  ss4.writeToFile(filenameA);

  /* construct the obstacles */
  scots::SymbolicSet obs4(ss4);
  vehicleCreateObstacles(obs4);
  sprintf(filenameB, "%s%d.bdd", "Obstacle", 4);
  obs4.writeToFile(filenameB);

  /* setup class for symbolic model computation */
  scots::SymbolicSet sspost4(ss4, 1);
  scots::SymbolicModelGrowthBound<state_type,input_type> abstraction4(&ss4, &is, &sspost4);
  std::cout << std::endl << "Interation 4" << std::endl;
  tt.tic();
  abstraction4.computeTransitionRelation(vehicle_post, radius_post);
  std::cout << std::endl;
  tt.toc();
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction4.getSize() << std::endl;
  std::cout << std::endl;

  /* controller synthesis */
  scots::SymbolicSet ts4(ss4);
  double H42[4*sDIM]={-1,  0, 0,
                       1,  0, 0,
                       0, -1, 0,
                       0,  1, 0};
  double h42[4] = {-6.5, 7, -6.25, 6.75};
  ts4.addPolytope(4, H42, h42, scots::INNER);
  sprintf(filename1, "%s%d.bdd", "Target", 5);
  ts4.writeToFile(filename1);

  scots::FixedPoint fp4(&abstraction4);
  BDD T4 = ts4.getSymbolicSet();
  BDD O4 = obs4.getSymbolicSet();
  tt.tic();
  BDD C4=fp4.reachAvoid(T4, O4, 1);
  tt.toc();

  scots::SymbolicSet controller4(ss4, is);
  controller4.setSymbolicSet(C4);
  sprintf(filename2, "%s%d.bdd", "Controller", 5);
  controller4.writeToFile(filename2);
  std::cout << std::endl;

/* Round 5 *************************************************************************************************/

  /* construct SymbolicSet for the state space */
  double lb5[sDIM]={4.6, 0.4, -M_PI};  
  double ub5[sDIM]={9.4, 7.6, M_PI}; 
  double eta5[sDIM]={0.18, 0.18, 0.18};   
  scots::SymbolicSet ss5(mgr, sDIM, lb5, ub5, eta5);
  ss5.addGridPoints();

  double H51[4*sDIM]={3, -4, 0,
                     -3, -4, 0,
                      3,  4, 0, 
                     -3,  4, 0};
  double h51[4] = {13.4, -29.6, 21.4, -20.6};
  ss5.remPolytope(4, H51, h51, scots::OUTER);

  sprintf(filenameA, "%s%d.bdd", "AbstractSet", 5);
  ss5.writeToFile(filenameA);

  /* construct the obstacles */
  scots::SymbolicSet obs5(ss5);
  vehicleCreateObstacles(obs5);
  sprintf(filenameB, "%s%d.bdd", "Obstacle", 5);
  obs5.writeToFile(filenameB);

  /* setup class for symbolic model computation */
  scots::SymbolicSet sspost5(ss5, 1);
  scots::SymbolicModelGrowthBound<state_type,input_type> abstraction5(&ss5, &is, &sspost5);
  std::cout << std::endl << "Interation 5" << std::endl;
  tt.tic();
  abstraction5.computeTransitionRelation(vehicle_post, radius_post);
  std::cout << std::endl;
  tt.toc();
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction5.getSize() << std::endl;
  std::cout << std::endl;

  /* controller synthesis 5-1 */
  scots::SymbolicSet ts51(ss5);
  double H52[4*sDIM]={-1,  0, 0,
                       1,  0, 0,
                       0, -1, 0,
                       0,  1, 0};
  double h52[4] = {-6, 6.5, -5, 5.5};
  ts51.addPolytope(4, H52, h52, scots::INNER);
  sprintf(filename1, "%s%d.bdd", "Target", 6);
  ts51.writeToFile(filename1);

  scots::FixedPoint fp51(&abstraction5);
  BDD T51 = ts51.getSymbolicSet();
  BDD O5 = obs5.getSymbolicSet();
  tt.tic();
  BDD C51=fp51.reachAvoid(T51, O5, 1);
  tt.toc();

  scots::SymbolicSet controller51(ss5, is);
  controller51.setSymbolicSet(C51);
  sprintf(filename2, "%s%d.bdd", "Controller", 6);
  controller51.writeToFile(filename2);
  std::cout << std::endl;

  /* controller synthesis 5-2 */
  scots::SymbolicSet ts52(ss5);
  double H53[4*sDIM]={-1,  0, 0,
                       1,  0, 0,
                       0, -1, 0,
                       0,  1, 0};
  double h53[4] = {-7, 7.5, -2, 2.5};
  ts52.addPolytope(4, H53, h53, scots::INNER);
  sprintf(filename1, "%s%d.bdd", "Target", 7);
  ts52.writeToFile(filename1);

  scots::FixedPoint fp52(&abstraction5);
  BDD T52 = ts52.getSymbolicSet();
  tt.tic();
  BDD C52 = fp52.reachAvoid(T52, O5, 1);
  tt.toc();

  scots::SymbolicSet controller52(ss5, is);
  controller52.setSymbolicSet(C52);
  sprintf(filename2, "%s%d.bdd", "Controller", 7);
  controller52.writeToFile(filename2);
  std::cout << std::endl;

/* Round 6 *************************************************************************************************/

  /* construct SymbolicSet for the state space */
  double lb6[sDIM]={4.2, 0.7, -M_PI};  
  double ub6[sDIM]={10, 4.3, M_PI}; 
  double eta6[sDIM]={0.15, 0.15, 0.15};   
  scots::SymbolicSet ss6(mgr, sDIM, lb6, ub6, eta6);
  ss6.addGridPoints(); 
  double H61[2*sDIM]={0.75, -1, 0,
                      0.75,  1, 0};
  double h61[2] = {0.65, 5.65};
  ss6.remPolytope(2, H61, h61, scots::OUTER);
  sprintf(filenameA, "%s%d.bdd", "AbstractSet", 6);
  ss6.writeToFile(filenameA);

  /* construct the obstacles */
  scots::SymbolicSet obs6(ss6);
  vehicleCreateObstacles(obs6);
  sprintf(filenameB, "%s%d.bdd", "Obstacle", 6);
  obs6.writeToFile(filenameB);

  /* setup class for symbolic model computation */
  scots::SymbolicSet sspost6(ss6, 1); 
  scots::SymbolicModelGrowthBound<state_type,input_type> abstraction6(&ss6, &is, &sspost6); 
  std::cout << std::endl << "Interation 6" << std::endl; 
  tt.tic(); 
  abstraction6.computeTransitionRelation(vehicle_post, radius_post); 
  std::cout << std::endl; 
  tt.toc(); 
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction6.getSize() << std::endl;
  std::cout << std::endl;

  /* controller synthesis */
  scots::SymbolicSet ts61(ss6);
  double H62[4*sDIM]={-1,  0, 0,
                       1,  0, 0,
                       0, -1, 0,
                       0,  1, 0};
  double h62[4] = {-9, 9.5, -1.5, 2};
  ts61.addPolytope(4, H62, h62, scots::INNER);
  sprintf(filename1, "%s%d.bdd", "Target", 8);
  ts61.writeToFile(filename1);

  scots::FixedPoint fp61(&abstraction6);
  BDD T61 = ts61.getSymbolicSet();
  BDD O6 = obs6.getSymbolicSet();
  tt.tic();
  BDD C61 = fp61.reachAvoid(T61, O6, 1);
  tt.toc();

  scots::SymbolicSet controller61(ss6, is);
  controller61.setSymbolicSet(C61);
  sprintf(filename2, "%s%d.bdd", "Controller", 8);
  controller61.writeToFile(filename2);
  std::cout << std::endl;

/* Safety ********************************************************************************************************/

  scots::SymbolicSet ts62(ss6);
  double H63[4*sDIM]={-1,  0, 0,
                       1,  0, 0,
                       0, -1, 0,
                       0,  1, 0};
  double h63[4] = {-9, 9.5, -1.5, 2};
  ts62.addPolytope(4, H63, h63, scots::INNER);
  sprintf(filename1, "%s%d.bdd", "Target", 9);
  ts62.writeToFile(filename1);
  scots::FixedPoint fp62(&abstraction6);
  BDD T62 = ts62.getSymbolicSet();

  tt.tic();
  size_t i,j;
  /* outer fp*/
  BDD X=mgr.bddOne();
  BDD XX=mgr.bddZero();
  /* inner fp*/
  BDD Y=mgr.bddZero();
  BDD YY=mgr.bddOne();
  /* the controller */
  BDD C=mgr.bddZero();
  BDD U=is.getCube();

  for(i=1; XX != X; i++) {
    X=XX;
    BDD preX=fp62.pre(X);
    /* init inner fp */
    YY = mgr.bddOne();
    for(j=1; YY != Y; j++) {
      Y=YY;
      YY= ( fp62.pre(Y) & T62 ) | preX;
    }
    XX=YY;
    std::cout << "Iterations inner: " << j << std::endl;
    BDD N = XX & (!(C.ExistAbstract(U)));
    C=C | N;
  }
  std::cout << "Iterations outer: " << i << std::endl;
  tt.toc();

  scots::SymbolicSet controller62(ss6, is);
  controller62.setSymbolicSet(C);
  std::cout << "Domain size: " << controller62.getSize() << std::endl;
  sprintf(filename2, "%s%d.bdd", "Controller", 9);
  controller62.writeToFile(filename2);
  std::cout << std::endl;

  return 1;

}

/****************************************************************************/
void vehicleCreateObstacles(scots::SymbolicSet &obs) {
  /* the obstacles are defined as polytopes H* x <= h */
  double H[4*sDIM]={-1,  0, 0,
                     1,  0, 0,
                     0, -1, 0,
                     0,  1, 0};
  /* add outer approximation of P={ x | H x<= h2 } form state space */
  double h1[4] = {-0, 2, -1, 3};
  obs.addPolytope(4, H, h1, scots::OUTER);
  double h2[4] = {-1, 3, -6.5, 8.5};
  obs.addPolytope(4, H, h2, scots::OUTER);
  double h3[4] = {-4, 5.8, -5, 8.5};
  obs.addPolytope(4, H, h3, scots::OUTER);
  double h4[4] = {-4, 6.5, -3.5, 5};
  obs.addPolytope(4, H, h4, scots::OUTER);
  double h5[4] = {-5, 6, -0, 2};
  obs.addPolytope(4, H, h5, scots::OUTER);  
  double h6[4] = {-6, 8, -0, 1};
  obs.addPolytope(4, H, h6, scots::OUTER);  
  double h7[4] = {-7, 9, -6, 7};
  obs.addPolytope(4, H, h7, scots::OUTER);
  double h8[4] = {-8, 10, -8, 10};
  obs.addPolytope(4, H, h8, scots::OUTER);  
  double h9[4] = {-8, 10, -2, 4};
  obs.addPolytope(4, H, h9, scots::OUTER);  
  double h10[4] = {-0, 2, -4.95, 5.05};
  obs.addPolytope(4, H, h10, scots::OUTER);
  double h11[4] = {-3, 7, -4.95, 5.05};
  obs.addPolytope(4, H, h11, scots::OUTER);  
  double h12[4] = {-8, 10, -4.95, 5.05};
  obs.addPolytope(4, H, h12, scots::OUTER);  
  double h13[4] = {-5.95, 6.05, -0, 5};
  obs.addPolytope(4, H, h13, scots::OUTER);  
  double h14[4] = {-3.95, 4.05, -5, 8.5};
  obs.addPolytope(4, H, h14, scots::OUTER);
}

/****************************************************************************/
scots::SymbolicSet vehicleCreateInputSpace(Cudd &mgr) {
  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={-1, -1};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={1, 1}; 
  /* grid node distance diameter */
  double eta[sDIM]={0.2, 0.2};   

  scots::SymbolicSet is(mgr, iDIM, lb, ub, eta);
  is.addGridPoints();

  return is;
}


