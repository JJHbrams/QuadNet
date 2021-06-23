/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr/terrain/examples/height_map_examples.h>
#include <random>
#include <math.h>

namespace towr {


FlatGround::FlatGround(double height)
{
  height_ = height;
}

double
Block::GetHeight (double x, double y) const
{
  double h = 0.0;

  // very steep ramp leading up to block
  if (block_start <= x && x <=block_start+eps_)
    h = slope_*(x-block_start);

  if (block_start+eps_ <= x && x <= block_start+length_)
    h = height_;

  return h;
}

double
Block::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

  // very steep ramp leading up to block
  if (block_start <= x && x <=block_start+eps_)
    dhdx = slope_;

  return dhdx;
}


// STAIRS
double
Stairs::GetHeight (double x, double y) const
{
  double h = 0.0;

  if (x>=first_step_start_)
    h = height_first_step;

  if (x>=first_step_start_+first_step_width_)
    h = height_second_step;

  if (x>=first_step_start_+first_step_width_+width_top)
    h = 0.0;

  return h;
}


// GAP
double
Gap::GetHeight (double x, double y) const
{
  double h = 0.0;

  // modelled as parabola
  if (gap_start_ <= x && x <= gap_end_x)
    h = a*x*x + b*x + c;

  return h;
}

double
Gap::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dhdx = 2*a*x + b;

  return dhdx;
}

double
Gap::GetHeightDerivWrtXX (double x, double y) const
{
  double dzdxx = 0.0;

  if (gap_start_ <= x && x <= gap_end_x)
    dzdxx = 2*a;

  return dzdxx;
}


// SLOPE
double
Slope::GetHeight (double x, double y) const
{
  double z = 0.0;
  if (x >= slope_start_)
    z = slope_*(x-slope_start_);

  // // going back down
  // if (x >= x_down_start_) {
  //   z = height_center - slope_*(x-x_down_start_);
  // }
  //
  // // back on flat ground
  // if (x >= x_flat_start_)
  //   z = 0.0;

  // flat height
  // if (x >= x_flat_start_)
  //   z = flat_height;

  return z;
}

double
Slope::GetHeightDerivWrtX (double x, double y) const
{
  double dzdx = 0.0;
  if (x >= slope_start_)
    dzdx = slope_;

  if (x >= x_down_start_)
    dzdx = -slope_;

  if (x >= x_flat_start_)
    dzdx = 0.0;

  return dzdx;
}


// Chimney
double
Chimney::GetHeight (double x, double y) const
{
  double z = 0.0;

  if (x_start_<=x && x<=x_end_)
    z = slope_*(y-y_start_);

  return z;
}

double
Chimney::GetHeightDerivWrtY (double x, double y) const
{
  double dzdy = 0.0;

  if (x_start_<= x && x<= x_end_)
    dzdy = slope_;

  return dzdy;
}


// Chimney LR
double
ChimneyLR::GetHeight (double x, double y) const
{
  double z = 0.0;

  if (x_start_<=x && x<=x_end1_)
    z = slope_*(y-y_start_);

  if (x_end1_<=x && x<=x_end2_)
    z = -slope_*(y+y_start_);

  return z;
}

double
ChimneyLR::GetHeightDerivWrtY (double x, double y) const
{
  double dzdy = 0.0;

  if (x_start_ <= x && x <= x_end1_)
    dzdy = slope_;

  if (x_end1_<=x && x<=x_end2_)
    dzdy = -slope_;

  return dzdy;
}

//***** Random terrain *****//
std::mt19937 generator;
double RAND_VAL(double MIN, double MAX){
  // Normaldistribution
  double mean = (MAX+MIN)/2;
  double stddev  = 1;
  std::normal_distribution<double> normal(mean, stddev);
  srand(normal(generator)*(time(NULL)));

  // std::cerr << "Normal: " << normal(generator) << std::endl;
  double RES = 1E4;
  int DIV = static_cast<int>((MAX-MIN)*RES+1);
  double SHIFT = ((MAX-MIN)/2.)*RES;
  double MEAN = (MAX+MIN)/2*RES;

  double output = (rand()%DIV - SHIFT + MEAN)/RES;

  return output;
}
/******************************************/
/********** Random Step Terrain ***********/
/******************************************/
// Random step Terrain
RandomStep::RandomStep()
{
  height_ = RAND_VAL(0., MAX_H);
  width_  = RAND_VAL(0.5, 3.5);//Step
  /*init_   = RAND_VAL(0.7, 2.5);
  init2_  = RAND_VAL(0.7, 2.5);*/
  init_   = RAND_VAL(0.5, 2.);
  init2_  = RAND_VAL(0.5, 2.);

  // Test randstep
  height_ = 0.5;
  width_ = 2.0;
  init_ = 0.5;
  init2_ = 1.5;

  // Nonflat
  // height_ = 0.2;
  // width_ = 2.0;
  init_ = 0.5;
  init2_ = 0.5;

}

double
RandomStep::GetHeight (double x, double y) const
{
  double h = 0.0;

  double hh=height_;
  double ww=width_;

  // hh=RAND_VAL(0.1, 0.2);
  // ww=RAND_VAL(0.5, 3.5);
  // double hh = std::abs(y/x)*0.04*(sin(abs(y/5*3)));
  // double ww = width_;

  // double slope_ = (init2_-init_)/4;
  // double terrain_start_ = slope_*(y+2)+init_;
  // double terrain_end_x = terrain_start_ + width_;

  double slope_ = (init2_-init_)/4;
  double terrain_start_ = slope_*(y+2)+init_;
  double terrain_end_x = terrain_start_ + ww;

  if (terrain_start_ <= x && x <= terrain_end_x)
    h = hh;

  return h;
}

/*****************************************/
/********** Random gap Terrain ***********/
/*****************************************/

// Random gap Terrain
RandomGap::RandomGap()
{
  // height_ = 1000;//Gap
  width_  = RAND_VAL(minGAP, MAXGAP);//Gap
  /*init_   = RAND_VAL(0.7, 2.5);
  init2_  = RAND_VAL(0.7, 2.5);*/
  init_   = RAND_VAL(0.5, 1.);
  init2_  = RAND_VAL(0.5, 1.);

  // Test random gap
  width_ = 0.2;
  init_ = 0.5;
  init2_ = 0.5;

}

void
RandomGap::Parabola(double &gs, double &ge, double&a, double&b, double&c, double curr) const{
  gs = (init2_-init_)/4.*(curr+2)+init_;
  ge = gs + width_;
  double xc = gs + dx; // gap center

  a = (height_)/(width_*width_);
  b = -2*a*xc;
  c = a*(xc*xc)-height_;
}

double
RandomGap::GetHeight (double x, double y) const
{
  double h = 0.0;
  double a,b,c;
  double gap_start_, gap_end_;

  Parabola(gap_start_,gap_end_,a,b,c,y);
  // if (gap_start_ <= x && x <= gap_end_)
  //   h = a*x*x + b*x + c;
  if (gap_start_ <= x && x <= gap_end_)
    h = -height_;

  return h;
}

double
RandomGap::GetHeightDerivWrtX (double x, double y) const
{
  double dhdx = 0.0;

  double a,b,c;
  double gap_start_, gap_end_;

  Parabola(gap_start_,gap_end_,a,b,c,y);
  if (gap_start_ <= x && x <= gap_end_)
    dhdx = 2*a*x + b;

  return dhdx;
}

double
RandomGap::GetHeightDerivWrtXX (double x, double y) const
{
  double dzdxx = 0.0;

  double a,b,c;
  double gap_start_, gap_end_;

  Parabola(gap_start_,gap_end_,a,b,c,y);
  if (gap_start_ <= x && x <= gap_end_)
    dzdxx = 2*a;

  return dzdxx;
}

/*******************************************/
/********** Random Slope Terrain ***********/
/*******************************************/
RandomSlope::RandomSlope()
{
  slope_ = RAND_VAL(0.1, 0.4);

  // Test randslope
  // slope_ = 0.2;

  height_ = width_*slope_;
}

double
RandomSlope::GetHeight (double x, double y) const
{
  double h = 0.0;
  if (x >= slope_start_)
    h = slope_*(x-slope_start_);

  return h;
}

double
RandomSlope::GetHeightDerivWrtX (double x, double y) const
{
  double dzdx = 0.0;
  if (x >= slope_start_)
    dzdx = slope_;

  return dzdx;
}

/**************************************/
/********** Nonflat Terrain ***********/
/**************************************/
NonFlat::NonFlat()
{
  height_ = RAND_VAL(0., MAX_H);
  width_  = RAND_VAL(0.5, 3.5);//Step
  /*init_   = RAND_VAL(0.7, 2.5);
  init2_  = RAND_VAL(0.7, 2.5);*/
  init_   = RAND_VAL(0.5, 2.);
  init2_  = RAND_VAL(0.5, 2.);
  // Nonflat
  init_ = 0.5;
  init2_ = 0.5;
}

double
NonFlat::GetHeight (double x, double y) const
{
  double h = 0.0;

  // double hh=height_;
  // double ww=width_;
  // hh=RAND_VAL(0.1, 0.2);
  // ww=RAND_VAL(0.5, 3.5);

  // double hh = std::abs(y/x)*0.04*(sin(abs(y/5*3)));
  // double hh = 0.0005*(std::pow(x,3)-3*x*std::pow(y,2))+0.2;
  double hh = 0.005*y*(y-x)*(y+x);
  double ww = 3.6;
  if(hh < 0)  hh = std::abs(y/(x+0.01))*0.04*(sin(abs(y/5*3)));//hh = -hh;
  // if(hh > 1)
  //   hh = 1;

  double slope_ = (init2_-init_)/4;
  double terrain_start_ = slope_*(y+2)+init_;
  double terrain_end_x = terrain_start_ + ww;

  // if (terrain_start_ <= x && x <= terrain_end_x)
  if (terrain_start_ <= x)
    h = hh;

  return h;
}


/*******************************************/
/********** Test step Terrain ***********/
/*******************************************/
TestStep::TestStep()
{
  height_ = RAND_VAL(0.2, 0.5);

}

double
TestStep::GetHeight (double x, double y) const
{
  double h = 0.0;

  double hh=height_;
  double ww=width_;

  double terrain_start_ = 3.0;
  double terrain_end_x = terrain_start_ + ww;

  if (terrain_start_ <= x && x <= terrain_end_x)
    h = hh;

  return h;
}


/*******************************************/
/********** Test Slope Terrain ***********/
/*******************************************/
TestSlope::TestSlope()
{
  slope_ = RAND_VAL(0.3, 0.7);

  // Test randslope
  // slope_ = 0.2;
  width_  = length_*std::cos(std::atan(slope_));
  height_ = width_*slope_;

  slope_end_ = slope_start_ + width_;
}

double
TestSlope::GetHeight (double x, double y) const
{
  double h = 0.0;

  if (x >= slope_start_)
    h = slope_*(x-slope_start_);

  if(x >= slope_end_)
    h = height_;

  return h;
}

double
TestSlope::GetHeightDerivWrtX (double x, double y) const
{
  double dzdx = 0.0;
  if (x >= slope_start_)
    dzdx = slope_;

  if (x >= slope_end_)
    dzdx = 0;

  return dzdx;
}


} /* namespace towr */
