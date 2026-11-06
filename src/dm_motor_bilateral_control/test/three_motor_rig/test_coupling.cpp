#include "coupling.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
using three_motor_rig::Coupling;
// Keep the numerical checks active in Release/NDEBUG builds too.
void require(bool condition)
{
  if (!condition) {throw std::runtime_error("Passive coupling numerical check failed");}
}

// Bare two-inertia fixture only validates link signs/passivity/integration.
// The production VirtualMotor benchmark separately includes motors and control.
double evolve(double dt)
{
  Coupling c{40, .02, 0, 0};
  double q1=.01,q3=0,v1=0,v3=0,loss=0;
  const double j1=.0039,j3=.0039,initial=c.evaluate(q1,v1,q3,v3).energy;
  for(int k=0;k<std::lround(1./dt);++k){
    const auto x=c.evaluate(q1,v1,q3,v3);
    // Midpoint integration of the same simultaneous coupling force.
    const double vm1=v1+.5*dt*x.torque1/j1, vm3=v3+.5*dt*x.torque3/j3;
    const auto mid=c.evaluate(q1+.5*dt*v1,vm1,q3+.5*dt*v3,vm3);
    q1+=dt*vm1;q3+=dt*vm3;v1+=dt*mid.torque1/j1;v3+=dt*mid.torque3/j3;
    loss+=dt*mid.dissipation;
  }
  const double total=c.evaluate(q1,v1,q3,v3).energy+.5*j1*v1*v1+.5*j3*v3*v3;
  require(total < initial && std::abs(q1-q3-.01)<1e-10);
  return std::abs(total+loss-initial)/initial;
}
int main() try
{
  Coupling c{40,.02,.001,0};
  for(double delta : std::array<double,6>{-.03,-.002,-.0005,.0005,.002,.03}){
    const double v1=.7,v3=-.3;
    const auto r=c.evaluate(delta,v1,0,v3);
    require(r.torque1==r.torque3 && r.dissipation>=0);
    const double eps=1e-8;
    const double derivative=(c.evaluate(delta+eps*(v1+v3),0,0,0).energy-
      c.evaluate(delta-eps*(v1+v3),0,0,0).energy)/(2*eps);
    require(std::abs(r.torque1*v1+r.torque3*v3+derivative+r.dissipation)<1e-8);
  }
  const auto free=c.evaluate(.0005,2,0,-1);require(free.torque1==0 && free.energy==0);
  const auto aligned=c.evaluate(.2,4,-.2,-4);require(aligned.torque1==0);
  const double coarse=evolve(1./32000),fine=evolve(1./64000);
  require(coarse<.0001 && fine<coarse*.4);
  std::cout << "passive link signs, backlash, power balance and half-step convergence passed; errors "
            << coarse << " " << fine << "\n";
}
catch (const std::exception & error)
{
  std::cerr << error.what() << '\n';
  return 1;
}
