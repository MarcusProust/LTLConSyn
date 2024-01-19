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

/* we integrate the vehicle ode by 0.3 sec (the result is stored in x)  */
auto  vehicle_post = [](state_type &x, input_type &u) -> void {
  auto rhs = [](state_type& xx, const state_type &x, input_type &u) {
      double alpha=std::atan(std::tan(u[1])/2.0);
      xx[0] = u[0]*std::cos(alpha+x[2])/std::cos(alpha);
      xx[1] = u[0]*std::sin(alpha+x[2])/std::cos(alpha);
      xx[2] = u[0]*std::tan(u[1]);
  };
  ode_solver(rhs, x, u);
};

/* computation of the growth bound (the result is stored in r)  */
auto radius_post = [](state_type &r, input_type &u) -> void {
    double c = std::abs(u[0]*std::sqrt(std::tan(u[1])*std::tan(u[1])/4.0+1));
    r[0] = r[0]+c*r[2]*0.2;
    r[1] = r[1]+c*r[2]*0.2;
};

scots::SymbolicSet vehicleCreateStateSpace(Cudd &mgr);
scots::SymbolicSet vehicleCreateInputSpace(Cudd &mgr);
void vehicleCreateObstacles(scots::SymbolicSet &obs);

/****************************************************************************/

int main() {
	
  double L[3][4] = {
         {-1.5, 2, -9, 9.5},
         {-6, 6.5, -5, 5.5},
         {-9, 9.5, -1.5, 2}
  };
  char filename1[256], filename2[256];

  TicToc tt;
  Cudd mgr;

  /* construct SymbolicSet for the state space */
  scots::SymbolicSet ss=vehicleCreateStateSpace(mgr);
  ss.writeToFile("AbstractSet.bdd");

  /* construct SymbolicSet for the obstacles */
  scots::SymbolicSet obs(ss);
  vehicleCreateObstacles(obs);
  obs.writeToFile("Obstacle.bdd");

  scots::SymbolicSet is=vehicleCreateInputSpace(mgr);

  /* setup class for symbolic model computation */
  scots::SymbolicSet sspost(ss, 1);
  scots::SymbolicModelGrowthBound<state_type,input_type> abstraction(&ss, &is, &sspost);
  tt.tic();
  abstraction.computeTransitionRelation(vehicle_post, radius_post);
  std::cout << std::endl;
  tt.toc();
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction.getSize() << std::endl;
  std::cout << std::endl;

  /* controller synthesis */
  for (int i=0; i<3; i++){
  scots::SymbolicSet ts(ss);

  double H[4*sDIM]={-1,  0, 0,
                     1,  0, 0,
                     0, -1, 0,
                     0,  1, 0};
  /* compute inner approximation of P={ x | H x<= h1 }  */
  double h[4] = {L[i][0], L[i][1], L[i][2], L[i][3]};
  ts.addPolytope(4, H, h, scots::INNER);
  sprintf(filename1, "%s%d.bdd", "Target", i+1);
  ts.writeToFile(filename1);

  scots::FixedPoint fp(&abstraction);
  BDD T = ts.getSymbolicSet();
  BDD O = obs.getSymbolicSet();
  tt.tic();
  BDD C=fp.reachAvoid(T, O, 1);
  tt.toc();

  scots::SymbolicSet controller(ss, is);
  controller.setSymbolicSet(C);
  sprintf(filename2, "%s%d.bdd", "Controller", i+1);
  controller.writeToFile(filename2);
  std::cout << std::endl;
}

/* Safety ******************************************************************************************************************************/
 
  scots::SymbolicSet ts32(ss);
  double HT32[4*sDIM]={-1,  0, 0,
                        1,  0, 0,
                        0, -1, 0,
                        0,  1, 0};
  double ht32[4] = {-9, 9.5, -1.5, 2};
  ts32.addPolytope(4, HT32, ht32, scots::INNER);
  sprintf(filename1, "%s%d.bdd", "Target", 4);
  ts32.writeToFile(filename1);

  scots::FixedPoint fp32(&abstraction);
  BDD T32 = ts32.getSymbolicSet();

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
    BDD preX=fp32.pre(X);
    /* init inner fp */
    YY = mgr.bddOne();
    for(j=1; YY != Y; j++) {
      Y=YY;
      YY= ( fp32.pre(Y) & T32 ) | preX;
    }
    XX=YY;
    std::cout << "Iterations inner: " << j << std::endl;
    BDD N = XX & (!(C.ExistAbstract(U)));
    C=C | N;
  }
  std::cout << "Iterations outer: " << i << std::endl;
  tt.toc();

  scots::SymbolicSet controller33(ss,is);
  controller33.setSymbolicSet(C);
  std::cout << "Domain size: " << controller33.getSize() << std::endl;
  sprintf(filename2, "%s%d.bdd", "Controller", 4);
  controller33.writeToFile(filename2);
  std::cout << std::endl;

  return 1;
}

/****************************************************************************/
scots::SymbolicSet vehicleCreateStateSpace(Cudd &mgr) {
  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={0, 0, -M_PI-0.4};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={10, 10, M_PI+0.4}; 
  /* grid node distance diameter */
  double eta[sDIM]={0.1, 0.1, 0.1};   

  scots::SymbolicSet ss(mgr, sDIM, lb, ub, eta);
  ss.addGridPoints();

 return ss;
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


