#include "cob_people_detection/eval_tool.h"


void EvalTool::config(std::string& name ,std::vector<double>& thresholds,int num_groups){
  thresh_=thresholds[0];
  p_.set_yrange(0,2000);
  p_.set_title(name);

}

void EvalTool::plot(std::vector<double>& y){


  std::vector<int> x;
  std::vector<int>    success_vec_x;
  std::vector<double> success_vec_y;

  // make plot vectors
  if(y.size()>0){
  for (int i = 0; i < y.size(); i++) {
    if(y[i]<thresh_){
      success_vec_y.push_back(y[i]);
      success_vec_x.push_back(i);
    }
    x.push_back(i);
  }

  //adapt axis ranges
  p_.set_xrange(0,y.size());
  p_.remove_tmpfiles();
  p_.reset_plot();

  // plot all distances
  p_.set_style("lines").plot_slope(0.0,thresh_,"UNKNOWN thresh");
  p_.set_pointsize(2).set_style("impulses").plot_xy(x,y,"misses");

  // plot succesfull matches
  if(success_vec_x.size()>0){
  p_.plot_xy(success_vec_x,success_vec_y,"success");
  }
  p_.showonscreen();
  }



}

int main(int argc, const char *argv[])
{

  return 0;
}
