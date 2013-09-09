#!/usr/bin/env python
import wx
import os
import math
import random
import sys
import subprocess

import math
from threading import Thread

"""
Grahical User Interface for database test.

To use this script a database has to be available with the following structure:

                                   DatabaseFolder
                                /     |              \
                      class_folder1  class_folder     class_folder

         c1_img0 c1_img1 c1_img2    c2_img0 c2_img1   c3_img0  c3_img1


To use this database select DatabaseFolder as Trainingset directory.
Either use one of the protocols that automatically divide the database in training data and test data or choose the test data manually.


####################################################################################################################
This script will produce the following ASCII files, that are supposed to be processed by the respective binary.

  - training_set_list : a list of paths to the files of the training set
  - probe_file_list : a list of pahts to the probe files
  - training_set_xyz_list : this list is filled with .xml paths to the
    respective xyz-pointclouds ( only filled, when xyz- processing is enabled via GUI)
  - probe_file_xyz_list : this list is filled with .xml paths to the
    respective xyz-pointclouds ( only filled, when xyz- processing is enabled via GUI)

  - class_overview : this list provides insight what class folder is assigned
    what class label

It takes as input the following ASCII files:
   - invalidlist : a list of paths, that is supposed to be excluded from
     evaluation
   - classification_labels : File with an integer classification for every
     probe file ( is needed for evaluation)

Evaluation results are provided via print command and are based on the  ASCII file:
   - eval_file : for single processing run
   - eval_file# : for every single processing run  - # is the number of
     processing run

####################################################################################################################
  To extend this script with an additional testing protocol (named <PROT>)you have to do the following:


    1. Add entry to self.protocol_choice in line 124 - this will give you the opportunity to select it on the GUI
    2. Add process_<PROT> function   (line 523)  - this function handles the processing sequence of the protocol and is handed over as a parameter to function process_protocol()
    3. Enable the process_<PROT> to be called from multiprocessing(line 393) and single processing(line 301).
    4. Add specific function <PROT> that handles  specific selection of test files etc.(line 697 ) - it is called from within the proces_<PROT> function


    Remark:If you are confused just look at the already implemented protocols,they are all structured in the same way.
    Maybe you should go from there..

"""

## Class contains a GUI that can handle database tests for a given binary
class dlg(wx.Frame):

  ## Constructor
  #
  # Path and binary definitions are set here
  def __init__(self):

    # varables for gui
    #----------------------------------------------
    # path where all output files are going to be stored
    self.base_path="/share/goa-tz/people_detection/eval/"
    # directory where the binary to be used is located
    self.bin_path="/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/bin/"
    # path to a  nonobligatory list with invalid filenames
    self.invalid_file_path="/share/goa-tz/people_detection/eval/eval_tool_files/invalidlist"
    # name of the executable
    self.bin_name="face_rec_alg_test"


    # fancy information 
    print "Database Evaluation GUI 2013 - ipa-goa-tz"
    print "running with binary: %s"%os.path.join(self.bin_path,self.bin_name)

    # Evaluator, that calculates numerical results
    self.Evaluator=Evaluator(invalid_list=self.invalid_file_path)


    # set output file directory and create it if necessary
    self.output_path=os.path.join(self.base_path,"eval_tool_files")
    if os.path.exists(self.output_path)==False:
      os.path.mkdir(self.output_path)

    # get current working directory
    self.cwd=os.getcwd()






    # variables for output
    self.pf_list = list()
    self.ts_list = list()
    self.ts_dir_list = list()
    self.cl_list = list()
    self.unknown_list=list()
    self.invalid_list=list()

    # can be switched to true if list synching has to be reversed this is only
    # necessary when DON'T USE
    self.synch_lists_switch=False


    # invalid file list is loaded, if it exists
    if os.path.isfile(self.invalid_file_path):
      with open(self.invalid_file_path,"r") as input_stream:
        self.invalid_list=input_stream.read().splitlines()
      print "invalid list loaded"
      print self.invalid_list


    # initialize GUI
    self.f=wx.Frame(None,title="Evaluation GUI",size=(650,500))
    self.makeLayout(self.f)
    self.makeBindings()
    self.f.Show()

  ## Function handles bindigs of buttons on GUI
  def makeBindings(self):
    self.dir_btn.Bind(wx.EVT_BUTTON,self.OnAddDir)
    self.pf_btn.Bind(wx.EVT_BUTTON,self.OnAddPf)
    self.ok_btn.Bind(wx.EVT_BUTTON,self.OnProcess)
    self.reset_btn.Bind(wx.EVT_BUTTON,self.OnReset)
    self.mproc_btn.Bind(wx.EVT_BUTTON,self.OnProcessMulti)

    self.del_dir_btn.Bind(wx.EVT_BUTTON,lambda evt,gl=self.ts_glist,l=self.ts_dir_list :self.OnResetList(evt,l,gl))
    self.del_pf_btn .Bind(wx.EVT_BUTTON,lambda evt,gl=self.pf_glist,l=self.pf_list :self.OnResetList(evt,l,gl))

  ## Function handles graphical setup of GUI
  def makeLayout(self,parent):
    pf_btn_txt=wx.StaticText(parent,-1,"Select probe file")
    self.pf_btn=wx.Button(parent,-1,"Browse",(70,30))

    del_pf_btn_txt=wx.StaticText(parent,-1,"Delete probe file")
    self.del_pf_btn=wx.Button(parent,-1,"Reset",(70,30))

    dir_btn_txt=wx.StaticText(parent,-1,"Add training set")
    self.dir_btn=wx.Button(parent,-1,"Browse",(70,30))

    del_dir_btn_txt=wx.StaticText(parent,-1,"Delete Database")
    self.del_dir_btn=wx.Button(parent,-1,"Reset",(70,30))

    reset_btn_txt=wx.StaticText(parent,-1,"Delete all lists")
    self.reset_btn=wx.Button(parent,-1,"Delete",(70,30))

    ok_btn_txt=wx.StaticText(parent,-1,"Processing")
    self.ok_btn=wx.Button(parent,-1,"Process",(70,30))

    protocol_choice_txt=wx.StaticText(parent,-1,"Testfile selection")
    self.protocol_choice=wx.Choice(parent,-1,choices=["leave one out","leave half out","manual selection","unknown","yale2","yale3","yale4","yale5","kinect","synth"])

     #  GENERIC TEST FUNCTION <PROT>
    #self.protocol_choice=wx.Choice(parent,-1,choices=["leave one out","leave half out","manual selection","unknown","yale2","yale3","yale4","yale5","kinect","synth","<PROT>"])

    self.spin_rep=wx.SpinCtrl(parent,-1,size=wx.Size(50,30),min=1,max=60)

    method_choice_txt=wx.StaticText(parent,-1,"Select Method")
    self.method_choice=wx.Choice(parent,-1,choices=["2D LDA","Fisherfaces","Eigenfaces","2D PCA"])

    self.nrm_checkbox=wx.CheckBox(parent,label="normalize")

    self.use_xyz_data=wx.CheckBox(parent,label="use xyz data")

    self.upd_checkbox=wx.CheckBox(parent,label="update lists")
    self.upd_checkbox.SetValue(True)


    mproc_btn_txt=wx.StaticText(parent,-1," Cross Validation")
    self.mproc_btn=wx.Button(parent,-1,"Process",(70,30))

    plist_btn_txt=wx.StaticText(parent,-1,"Print lists")
    self.plist_btn=wx.Button(parent,-1,"Print",(70,30))

    # visual feedback lists
    self.ts_glist=wx.ListBox(choices=[],id=-1,parent=parent,size=wx.Size(80,100))
    self.pf_glist=wx.ListBox(choices=[],id=-1,parent=parent,size=wx.Size(80,100))

    # dummy for filling empty spaces
    dummy=wx.StaticText(parent,-1,'')

    # Flexgridsizer handles alignment of graphical elements
    sizer=wx.FlexGridSizer(10,3,0,0)
    sizer.AddGrowableCol(2)

    sizer.Add(dir_btn_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(del_dir_btn_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(dummy,1,wx.EXPAND)

    bs_3=wx.BoxSizer(wx.VERTICAL)
    bs_3.Add(self.dir_btn,1)
    bs_3.Add(self.use_xyz_data,1)
    sizer.Add(bs_3,1)
    sizer.Add(self.del_dir_btn,1)
    sizer.Add(self.ts_glist,1,wx.EXPAND)

    sizer.Add(pf_btn_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(del_pf_btn_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(dummy,1,wx.EXPAND)

    sizer.Add(self.pf_btn,1)
    sizer.Add(self.del_pf_btn,1)
    sizer.Add(self.pf_glist,1,wx.EXPAND)

    sizer.Add(protocol_choice_txt,1,wx.BOTTOM |wx.ALIGN_BOTTOM)
    sizer.Add(dummy,1,wx.EXPAND)
    sizer.Add(dummy,1,wx.EXPAND)

    sizer.Add(self.protocol_choice,1)
    sizer.Add(dummy,1,wx.EXPAND)
    sizer.Add(dummy,1,wx.EXPAND)

    sizer.Add(ok_btn_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)
    sizer.Add(dummy,1,wx.EXPAND)
    sizer.Add(method_choice_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)

    sizer.Add(self.ok_btn,1)
    sizer.Add(self.upd_checkbox,1)

    bs=wx.BoxSizer(wx.HORIZONTAL)
    bs.Add(self.method_choice,1)
    bs.Add(self.nrm_checkbox,1)
    sizer.Add(bs,1,wx.BOTTOM | wx.ALIGN_BOTTOM)

    sizer.Add(mproc_btn_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)
    sizer.Add(dummy,1,wx.EXPAND)
    sizer.Add(dummy,1,wx.EXPAND)

    sizer.Add(self.mproc_btn,1)
    sizer.Add(self.spin_rep,1)
    sizer.Add(dummy,1,wx.EXPAND)

    sizer.Add(reset_btn_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)
    sizer.Add(plist_btn_txt,1,wx.BOTTOM | wx.ALIGN_BOTTOM)
    sizer.Add(dummy,1,wx.EXPAND)

    sizer.Add(self.reset_btn,1)
    sizer.Add(self.plist_btn,1)
    sizer.Add(dummy,1,wx.EXPAND)

    parent.SetSizer(sizer)


# Callback methods

  ## Callback function when evaluation is supposed to take place.
  def OnEvaluate(self,e):
    self.evaluate()

  ## Callback function for button to add probe files.
  def OnAddPf(self,e):
    self.pf_dlg=wx.FileDialog(None,"Select Probefile",defaultDir=self.base_path,style=wx.FD_MULTIPLE)
    if self.pf_dlg.ShowModal() == wx.ID_OK:
      temp_list= self.pf_dlg.GetPaths()
      for temp in temp_list:
        self.pf_glist.Append(temp)

  ## Callback function for button to add database directory.
  def OnAddDir(self,e):
    self.ts_dlg=wx.DirDialog(None,"Select directories")
    self.ts_dlg.SetPath(self.base_path)
    if self.ts_dlg.ShowModal() == wx.ID_OK:
      self.ts_dir_list.append(self.ts_dlg.GetPath())
      self.ts_glist.Append(self.ts_dir_list[-1])

  ## Callback function for button with multiple processing runs
  def OnProcessMulti(self,e):

    # retrieve configuration information from GUI
    method=str()
    xyz_tag=str()
    if self.method_choice.GetCurrentSelection()==0:
      method="LDA2D"
    elif self.method_choice.GetCurrentSelection()==1:
      method="FISHER"
    elif self.method_choice.GetCurrentSelection()==2:
      method="EIGEN"
    elif self.method_choice.GetCurrentSelection()==3:
      method="PCA2D"

    if self.use_xyz_data.Value==True:
      xyz_tag="1"
    elif self.use_xyz_data.Value==False:
      xyz_tag="0"

    # The test protocol is chosen.
    # Every test protocol runs a different kind of test with the specified
    # data.
    # New protocols can be called here.
    prot_choice=self.protocol_choice.GetCurrentSelection()
    for i in xrange(self.spin_rep.GetValue()):
      output_file=os.path.join(self.output_path,"eval_file")
      if len(self.ts_dir_list)>0:
          # use leave 1 out protocol
        if(self.protocol_choice.GetCurrentSelection()==0):
          self.process_protocol(self.process_leave_1_out,method)

        # use leave half out protocol
        elif(self.protocol_choice.GetCurrentSelection()==1):
          self.process_protocol(self.process_leave_half_out,method)

        elif(self.protocol_choice.GetCurrentSelection()==2):
          print "manual selection not available for cross validation"
          return
        # use "unknown" protocol
        elif(self.protocol_choice.GetCurrentSelection()==3):
          self.process_protocol(self.process_unknown,method)
        # use yale protocoll with subset 2
        elif(prot_choice==4):
          self.yale_flag=2
          self.process_protocol(self.process_yale(2),method)
        # use yale protocoll with subset 3
        elif(prot_choice==5):
          self.yale_flag=3
          self.process_protocol(self.process_yale(3),method)
        # use yale protocoll with subset 4
        elif(prot_choice==6):
          self.yale_flag=4
          self.process_protocol(self.process_yale(4),method)
        # use yale protocoll with subset 5
        elif(prot_choice==7):
          self.yale_flag=5
          self.process_protocol(self.process_yale(5),method)
        # process synth -- DONT USE
        elif(prot_choice==9):
          self.process_protocol(self.process_synth,method)

        # GENERIC TESTING PROTOCOL <PROT>
        #elif(prot_choice==10):
        #  self.process_protocol(self.process_<PROT>,method)

        # run binary
        os.chdir(self.bin_path)
        if self.nrm_checkbox.GetValue()==True:
          normalizer="1"
        else:
          normalizer="0"
        t=Thread(target=self.run_bin,args=(method,normalizer,xyz_tag))
        t.start()
        t.join()

        os.chdir(self.cwd)
        # run evaluation
        self.evaluate()
        # rename output files according to number of current run
        os.rename(output_file,output_file+str(i))

    # show accumulated stats
    print self.Evaluator.calc_stats()
    # reset evaluator for next processing
    self.Evaluator.reset()


  ## Callback function which is used when single processing run is necessary
  def OnProcess(self,e):
    # get configuration from GUI
    method=str()
    classifier=str()
    xyz_tag=str()
    if self.method_choice.GetCurrentSelection()==0:
      method="LDA2D"
    elif self.method_choice.GetCurrentSelection()==1:
      method="FISHER"
    elif self.method_choice.GetCurrentSelection()==2:
      method="EIGEN"
    elif self.method_choice.GetCurrentSelection()==3:
      method="PCA2D"

    # The test protocol is chosen.
    # Every test protocol runs a different kind of test with the specified
    # data.
    # New protocols can be called here.

    prot_choice=self.protocol_choice.GetCurrentSelection()
    # if lists are supposed to be updated - for the case of random sampling
    if self.upd_checkbox.GetValue()==True:
      if len(self.ts_dir_list)>0:
        # leave 1 out protocol
        if(self.protocol_choice.GetCurrentSelection()==0):
          self.process_protocol(self.process_leave_1_out,method)
          output_file=os.path.join(self.output_path,"eval_file")
          self.process_protocol(self.process_leave_1_out,method)

        # leave half out protocol
        elif(self.protocol_choice.GetCurrentSelection()==1):
          self.process_protocol(self.process_leave_half_out,method)
          output_file=os.path.join(self.output_path,"eval_file")
          self.process_protocol(self.process_leave_half_out,method)

        # manual test file selection
        elif(self.protocol_choice.GetCurrentSelection()==2):
          self.process_protocol(self.process_manual,method)
        # "unknown" protocol
        elif(self.protocol_choice.GetCurrentSelection()==3):
          output_file=os.path.join(self.output_path,"eval_file")
          self.process_protocol(self.process_unknown,method)
        # yale protocol with subset 2 for testing
        elif(prot_choice==4):
          self.yale_flag=2
          self.process_protocol(self.process_yale,method)
        # yale protocol with subset 3 for testing
        elif(prot_choice==5):
          self.yale_flag=3
          self.process_protocol(self.process_yale,method)
        # yale protocol with subset 4 for testing
        elif(prot_choice==6):
          self.yale_flag=4
          self.process_protocol(self.process_yale,method)
        # yale protocol with subset 5 for testing
        elif(prot_choice==7):
          self.yale_flag=5
          self.process_protocol(self.process_yale,method)
        # kinect protocoll for RGB-D database
        elif(prot_choice==8):
          self.process_protocol(self.process_kinect,method)
        # synth faces protocol DON't USE
        elif(prot_choice==9):
          self.process_protocol(self.process_synth,method)

        #  GENERIC TEST FUNCTION <PROT>
        #elif(prot_choice==10):
          #self.process_protocol(self.process_<PROT>,method)



    if self.use_xyz_data.Value==True:
      xyz_tag="1"
    elif self.use_xyz_data.Value==False:
      xyz_tag="0"

    # run binary
    os.chdir(self.bin_path)
    if self.nrm_checkbox.GetValue()==True:
      normalizer="1"
    else:
      normalizer="0"
    t=Thread(target=self.run_bin,args=(method,normalizer,xyz_tag))
    t.start()
    t.join()


    os.chdir(self.cwd)
    # evaluate results
    self.evaluate()

    # show calculatet stats
    print self.Evaluator.calc_stats()
    # reset Evaluator for next run
    self.Evaluator.reset()

  def OnReset(self,e):
      self.delete_files()

  def OnResetList(self,e,l,gl):
    del l[:]
    gl.Clear()

  ## Function deletes output files.
  def delete_files(self):
    os.chdir(self.output_path)
    str_list=["rm","classification_labels","eval_file","class_overview","probe_file_list","probe_file_xyz_list","training_set_list","training_set_xyz_list"]
    subprocess.call(str_list)


  def reset_lists(self):
    del self.ts_list[:]
    del self.pf_list[:]
    del self.cl_list[:]
    del self.unknown_list[:]


#*****************************************************
#****************Internal Functions********************
#*****************************************************

  ## This function handles the generic test protocol processing and calls the specific protocol functions.
  def process_protocol(self,protocol_fn,method):

    self.reset_lists()
    # make training set list and class list
    self.file_ops(self.make_ts_list)
    self.file_ops(self.make_cl_list)
    # call test protocol specific function
    protocol_fn()
    # make sure no file is in test and training data at the same time
    self.sync_lists()
    #print self.ts_list
    #print "--------------------------------------------------------"
    #print self.pf_list
    self.print_lists()


  ## Function runs binary with specified configuration
  def run_bin(self,method,normalize,xyz):
    binary=os.path.join(self.bin_path,self.bin_name)
    subprocess.call([binary,method,normalize,xyz])

  # Specific protocols
  def process_synth(self):
    self.process_leave_1_out()
    tmp=self.pf_list
    self.pf_list=self.ts_list
    self.pf_list.append([])
    self.ts_list=tmp
    del self.ts_list[-1]
    self.synch_lists_switch=True


  ## test protocol for RGB-D database
  def process_kinect2(self):
    self.ts_list=list()
    self.pf_list=list()
    self.file_ops(self.kinect2)
    ##append empty list for unknown calssifications
    self.pf_list.append([])
    self.synch_lists_switch=False

  ## test protocol for RGB-D database
  def process_kinect(self):
    self.reset_lists()
    pf_path=os.path.join(self.base_path,"eval_tool_files","pf_list")
    ts_path=os.path.join(self.base_path,"eval_tool_files","ts_list")
    curr_files=list()
    with open(ts_path,"r") as input_stream:
      lines=input_stream.read().splitlines()
      for line in lines:
        if "$$" in line:
          self.ts_list.append(curr_files)
          curr_files=list()
        else:
          curr_files.append(line)


    cl_ctr=0
    curr_files=list()
    with open(pf_path,"r") as input_stream:
      lines=input_stream.read().splitlines()
      for line in lines:
        if "$$" in line:
          print line
          cl_ctr+=1
          self.cl_list.append(cl_ctr)
          self.pf_list.append(curr_files)
          curr_files=list()
        else:
          curr_files.append(line)


    self.pf_list.append([])
    self.synch_lists_switch=False
    self.sync_lists()
    self.print_lists()


# GENERIC TEST FUNCTION <PROT>
  #def process_<PROT>(self):
    ## this function handles the selection of test files from the training
    ## database


  ## test protocol for Yale database
  def process_yale(self):
    k=self.yale_flag
    self.file_ops(self.yale,k)
    ##append empty list for unknown calssifications
    self.pf_list.append([])
    self.synch_lists_switch=False

  ## test protocol for unknown tests
  def process_unknown(self):
    C=len(self.cl_list)
    ctr=0
    rnd_list=list()
    k=[False for i in range(int(len(self.cl_list)))]
    while ctr < math.floor(C*0.5):
      rnd=random.randint(0,C-1)
      if  rnd not in  rnd_list:
        rnd_list.append(rnd)
        k[rnd]=True
        ctr+=1
    i=0
    for cl in self.cl_list:
      if cl:
        del self.cl_list[i]
      i+=1

    self.file_ops(self.unknown,k)
    self.pf_list.append(self.unknown_list)
    del k[:]
    del rnd_list[:]
    self.synch_lists_switch=False

  ## test protocol for  leave half out protocol
  def process_leave_half_out(self):
    self.file_ops(self.leave_k_out,"half")
    ##append empty list for unknown calssifications
    self.pf_list.append([])
    self.synch_lists_switch=False

  ## test protocol for leave 1 out protocol
  def process_leave_1_out(self):
    self.file_ops(self.leave_k_out,1)
    ##append empty list for unknown calssifications
    self.pf_list.append([])
    self.synch_lists_switch=False

  ## test protocol for manual probe file selection
  def process_manual(self):
    self.pf_list=[[] for i in range(len(self.cl_list)+1)]
    self.pf_list_format(self.pf_glist.GetItems())
    self.synch_lists_switch=False

  ## Funcion handles operations with specific filter function as parameter
  def file_ops(self,fn=-1,add_param=False):

    # when default value is given
    if fn ==-1:
      def fn(x):
        return x

    for db in self.ts_dir_list:
      db_path=db
      os.chdir(db_path)

      dir_list=os.listdir(".")
      # enter directory - class
      for dir in dir_list:
        file_list_valid=list()
        os.chdir(dir)
        file_list_all=os.listdir(".")

        # loop through all files
        for file in file_list_all:
          # filter files for known image formats
          if file.endswith(".bmp") or file.endswith(".jpg") or file.endswith(".pgm") or file.endswith(".png"):
            if not file.endswith("Ambient.pgm") or file in self.invalid_list:

              # construct filepath
              file_path=db_path+"/"+dir+"/"+file
              file_list_valid.append(file_path)
        if add_param==False:
          fn(file_list_valid)
        else:
          fn(file_list_valid,add_param)
        os.chdir(db_path)


  def pf_list_format(self,file_list):
    for cl in xrange(len(self.ts_list)):
        for i in reversed(xrange(len(file_list))):
          if file_list[i] in self.ts_list[cl]:
            self.pf_list[cl].append(file_list[i])
            file_list.remove(file_list[i])
    for rf in file_list:
      self.pf_list[-1].append(rf)

#  protocol specific filter functions
  def kinect(self,files):
    ts=list()
    pf=list()
    for f in files:
      name=os.path.split(f)[1]
      if name[0]=="_":
        #print name
        pf.append(f)
      else:
        ts.append(f)
    self.pf_list.append(pf)
    self.ts_list.append(ts)

  def kinect2(self,files):
    ts_list=list()
    pf_list=list()
    for file in files:
      persp=(int(file[-8]))
      if (persp == 7 or
          persp == 11 or
          persp == 13 or
          persp == 11 or
          persp == 1
          ):
        ts_list.append(file)
      else:
        pf_list.append(file)
    self.pf_list.append(pf_list)
    self.ts_list.append(ts_list)

  def yale(self, files,k):


     ss=[[] for i in range(5)]
     for item in files:
       if (int(item[-11:-8]) <=12 and int(item[-6:-4]) <=12):
         ss[0].append(item)
         continue
       elif (int(item[-11:-8]) <=25 and  int(item[-6:-4]) <=25):
         ss[1].append(item)
         continue
       elif (int(item[-11:-8]) <=50 and int(item[-6:-4]) <=50):
         ss[2].append(item)
         continue
       elif (int(item[-11:-8]) <=77 and int(item[-6:-4]) <=77):
         ss[3].append(item)
         continue
       elif ( int(item[-11:-8]) >77 or int(item[-6:-4]) >77):
         ss[4].append(item)
         continue

     self.pf_list.append(ss[k-1])
     for s in ss[1:]:
       for f in s:
           for c in self.ts_list:
             if f in c:
               c.remove(f)


  #GENERIC TEST PROTOCOL <PROT>
  #def <PROT>(self,files):

  def unknown(self, files,k):
      if k[0] ==True:
        self.unknown_list.extend(files)
        del k[0]
      else:
        num_samples=len(files)
        n=num_samples/2
        #n=9
        success_ctr=0
        rnd_list=list()
        pf_list=list()
        while len(rnd_list)<n:
          rnd_no=random.randint(0,num_samples-1)
          if not rnd_no in rnd_list :
            pf_list.append(files[rnd_no])
            rnd_list.append(rnd_no)
        self.pf_list.append(pf_list)
        del k[0]

  def leave_k_out(self,file_list_valid,k):
        num_samples=len(file_list_valid)
        if(k is "half"):
          k=num_samples/2
        success_ctr=0
        rnd_list=list()
        pf_list=list()
        while len(rnd_list)<k:
          rnd_no=random.randint(0,num_samples-1)
          if not rnd_no in rnd_list :
            pf_list.append(file_list_valid[rnd_no])
            rnd_list.append(rnd_no)
        self.pf_list.append(pf_list)

  def make_ts_list(self,file_list):
      self.ts_list.append(file_list)

  def make_cl_list(self,file_list):
    class_name=os.path.basename(os.getcwd())
    self.cl_list.append(class_name)


  def sync_lists(self):
    if  self.synch_lists_switch ==True:
      for c in xrange(len(self.ts_list)):
        for s in self.ts_list[c]:
          if len(s) >0:
            for ts in self.pf_list:
              if s in ts:
                ts.remove(s)
    else:
      for c in xrange(len(self.pf_list)):
        for s in self.pf_list[c]:
          if len(s) >0:
            for ts in self.ts_list:
              if s in ts:
                ts.remove(s)



  def evaluate(self):

    results=list()
    groundtruth=list()
    files=list()

    input_path=os.path.join(self.output_path,"classification_labels")
    eval_file_path=os.path.join(self.output_path,"eval_file")
    with open(input_path,"r") as input_stream:
      classified_list=input_stream.read().splitlines()

    with open(eval_file_path,"w") as eval_file_stream:
      cont_ctr=0
      for pf_l in xrange(len(self.cl_list)):
        for pf in self.pf_list[pf_l]:
          o_str = pf+" "+str(pf_l)+" "+str(classified_list[cont_ctr])+"\n"
          eval_file_stream.write(o_str)

          results.append(classified_list[cont_ctr])
          groundtruth.append(pf_l)
          files.append(pf)

          cont_ctr+=1
      # append unknown probe files as -1
      for ukf in self.pf_list[-1]:
            o_str = ukf+" "+str(-1)+" "+str(classified_list[cont_ctr])+"\n"
            eval_file_stream.write(o_str)
            results.append(classified_list[cont_ctr])
            groundtruth.append(-1)
            files.append(ukf)
            cont_ctr+=1

    self.Evaluator.add_epoch(groundtruth,results,files)
    print "error rate = %f"%self.Evaluator.show_last()


  def print_lists(self):
    #print self.pf_list

    #print "[EVAL TOOL] creating lists"
    training_set_list_path=os.path.join(self.output_path,"training_set_list")
    training_set_list_xyz_path=os.path.join(self.output_path,"training_set_xyz_list")
    probe_file_list_path=os.path.join(self.output_path,"probe_file_list")
    probe_file_xyz_list_path=os.path.join(self.output_path,"probe_file_xyz_list")
    class_overview_path=os.path.join(self.output_path,"class_overview")


    training_set_file_stream = open(training_set_list_path,"w")
    training_set_file_xyz_stream = open(training_set_list_xyz_path,"w")
    probe_file_stream = open(probe_file_list_path,"w")
    probe_file_xyz_stream = open(probe_file_xyz_list_path,"w")
    class_overview_stream = open(class_overview_path,"w")


    # make lists with depth
    if self.use_xyz_data.Value:
      for c in xrange(len(self.ts_list)):
        if len(self.ts_list[c])>0:
          for s in self.ts_list[c]:
              if os.path.split(s)[1] not in self.invalid_list:
                s_mod=os.path.splitext(s)[0]+".xml"
                training_set_file_xyz_stream.write(s_mod)
                training_set_file_xyz_stream.write("\n")
          training_set_file_xyz_stream.write("$$\n")


      for c in xrange(len(self.pf_list)):
        for s in self.pf_list[c]:
          if os.path.split(s)[1] not in self.invalid_list:
            s_mod=os.path.splitext(s)[0]+".xml"
            probe_file_xyz_stream.write(s_mod)
            probe_file_xyz_stream.write("\n")

    for c in xrange(len(self.ts_list)):
      if len(self.ts_list[c])>0:
        for s in self.ts_list[c]:
            if os.path.split(s)[1] not in self.invalid_list:
              training_set_file_stream.write(s)
              training_set_file_stream.write("\n")
        training_set_file_stream.write("$$\n")

    for c in xrange(len(self.pf_list)):
      for s in self.pf_list[c]:
        if os.path.split(s)[1] not in self.invalid_list:
          probe_file_stream.write(s)
          probe_file_stream.write("\n")

    for c in xrange(len(self.cl_list)):
      o_str=str(c)+" - "+str(self.cl_list[c])+"\n"
      class_overview_stream.write(o_str)

#---------------------------------------------------------------------------------------
#----------------------------EVALUATOR-----------------------------------------
#---------------------------------------------------------------------------------------

### Class Epoch to be used with evaluator
class epoch():
  def __init__(self,gt,res,desc,ctr):
    self.gt=gt
    self.res=res
    self.desc=desc
    self.error_rate=float()

    self.calc_error_rate()

  def print_error_list(self,error_list_path):
    error_list_stream = open(error_list_path,"w")
    for s in self.fi_list:
      o_str=str(s)+" \n"
      error_list_stream.write(o_str)

  def calc_error_rate(self):
    error_list=list()
    #true positive
    tp_list=list()
    #true negative
    tn_list=list()
    #false positive
    fp_list=list()
    #false negative
    fn_list=list()
    #false id
    self.fi_list=list()
    for i in xrange(len(self.gt)):
        if (int(self.gt[i]) == int(self.res[i])):
          error_list.append(0)
          if int(self.gt[i]) >-1:
            fp_list.append(0)
            fn_list.append(0)
            tn_list.append(0)
            tp_list.append(1)
            self.fi_list.append(0)
          elif int(self.gt[i]) ==-1:
            fp_list.append(0)
            fn_list.append(0)
            tn_list.append(1)
            tp_list.append(0)
            self.fi_list.append(0)
        else:
          error_list.append(1)
          if int(self.gt[i])==-1:
            fp_list.append(1)
            fn_list.append(0)
            tn_list.append(0)
            tp_list.append(0)
            self.fi_list.append(0)
          elif int(self.gt[i]) >-1 and int(self.res[i])==-1:
            fp_list.append(0)
            fn_list.append(1)
            tn_list.append(0)
            tp_list.append(0)
            self.fi_list.append(0)
          elif int(self.gt[i]) >-1 and int(self.res[i])>-1:
            self.fi_list.append(1)

    n=len(error_list)
    self.error_rate=float(sum(error_list))/float(n)
    self.tp=sum(tp_list)
    self.tn=sum(tn_list)
    self.fp=sum(fp_list)
    self.fn=sum(fn_list)
    self.fi=sum(self.fi_list)


### Class Evaluator
class Evaluator():
  def __init__(self,invalid_list=0):
    self.error_list_path="/share/goa-tz/people_detection/eval/eval_tool_files/error_list"
    if invalid_list==0:
      self.invalid_files=list()
    else:
      if os.path.exists(invalid_list):
        with open(invalid_list,"r") as input_stream:
            self.invalid_files=input_stream.read().splitlines()
      else:
        self.invalid_files=list()

    self.epochs=list()
    self.epoch_ctr=0

  def reset(self):
    self.epoch_ctr=0
    del self.epochs[:]


  def add_epoch(self,gt,res,files):
    e=epoch(gt,res,files,self.epoch_ctr)
    e.print_error_list(self.error_list_path)
    self.epoch_ctr+=1
    self.epochs.append(e)

  def calc_stats(self):
    n=float(len(self.epochs))
    err=0.0
    mean_error_rate=0.0
    for e in self.epochs:
      if e.desc in self.invalid_files:
        print "invalid file is excluded from statistical calculations"
        continue
      err+=e.error_rate
    mean_error_rate=err/n


    if len(self.epochs)>1:
      v2=0.0
      for e in self.epochs:
        v2+=(e.error_rate - mean_error_rate)*(e.error_rate - mean_error_rate)
      sigma=math.sqrt(1/(n-1)*v2)
    else:
      sigma=0.0


   # #remove comment to display true positive and fals positive

    self.print_ave_values(self.epochs)
    stats={"succes_rate":1-mean_error_rate,"m_err":mean_error_rate,"sigma":sigma,"reps":len(self.epochs)}


    return stats

  def show_all(self):
    err=list()
    for e in self.epochs:
      err.append(e.error_rate)

    return err

  def print_ave_values(self,l):
    ave_tp=0.0
    ave_tn=0.0
    ave_fp=0.0
    ave_fn=0.0
    ave_fi=0.0
    for el in l:
      ave_tp+=el.tp
      ave_tn+=el.tn
      ave_fp+=el.fp
      ave_fn+=el.fn
      ave_fi+=el.fi
    ave_tp/=len(l)
    ave_tn/=len(l)
    ave_fp/=len(l)
    ave_fn/=len(l)
    ave_fi/=len(l)

    print "True positives:   %f"%   ave_tp
    print "True negatives:   %f"%   ave_tn
    print "False positives:  %f"%  ave_fp
    print "False negatives:  %f"%  ave_fn
    print "False identities: %f"% ave_fi

  def show_last(self):
    err=self.epochs[-1].error_rate

    return err




if __name__=="__main__":
  app= wx.App(False)
  dlg = dlg()
  app.MainLoop()
