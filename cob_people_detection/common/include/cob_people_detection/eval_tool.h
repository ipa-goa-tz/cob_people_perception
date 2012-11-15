#include<iostream>
#include <fstream>
#include "gnuplot/gnuplot_i.hpp"

class EvalTool
{
  public:
  EvalTool(){};
  ~EvalTool(){};

  void plot(std::vector<double>& y);
  void config(std::string& name ,std::vector<double>& thresholds,int num_groups);

  protected:
  Gnuplot p_;
  double thresh_;

};
