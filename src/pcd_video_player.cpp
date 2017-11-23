/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 * 
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

//PCL
#include "include/pcd_video_player.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//STD
#include <iostream>
#include <fstream>

//BOOST
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

//QT
#include <QApplication>
#include <QMutexLocker>
#include <QEvent>
#include <QObject>
#include <QFileDialog>
#include <QGroupBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QShortcut>

// VTK
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>
#include <boost/smart_ptr/make_shared.hpp>


using namespace pcl;
using namespace std;


////////////////////////////////////////////////////////////////////////////////
PCDVideoPlayer::PCDVideoPlayer () {
  cloud_present_ = false;
  cloud_modified_ = false;
  play_mode_ = false;
  speed_counter_ = 0;
  speed_value_ = 5;

  //Create a timer
  vis_timer_ = new QTimer (this);
  vis_timer_->start (33);//5ms

  connect (vis_timer_, SIGNAL (timeout ()), this, SLOT (timeoutSlot ()));

  ui_ = new Ui::MainWindow;
  ui_->setupUi (this);

  this->setWindowTitle ("PCL PCD Video Player");

  // Setup the cloud pointer
  cloud_ = boost::make_shared <Cloud> ();

  // Set up the qvtk window
  vis_ = boost::make_shared <pcl::visualization::PCLVisualizer> ("", false);
  vis_->setBackgroundColor (0.8, 0.8, 0.8);
  vis_->setSize (320, 240);
  vis_->initCameraParameters ();
  vis_->setCameraFieldOfView (1.02259994 / (640.0 / 480.0));
  vis_->setShowFPS (false);
  ui_->qvtkWidget->SetRenderWindow (vis_->getRenderWindow ());
  vis_->setupInteractor (ui_->qvtkWidget->GetInteractor (),
                         ui_->qvtkWidget->GetRenderWindow ());
  vis_->getInteractorStyle ()->setKeyboardModifier (
      pcl::visualization::INTERACTOR_KB_MOD_SHIFT);


  ui_->qvtkWidget->update ();

  new QShortcut (QKeySequence (Qt::Key_S),
                 this,
                 SLOT (snapshotButtonPressed ()));

  // Connect player controls
  connect (ui_->playButton,
           SIGNAL (clicked ()),
           this,
           SLOT (playButtonPressed ()));

  connect (ui_->stopButton,
           SIGNAL (clicked ()),
           this,
           SLOT (stopButtonPressed ()));

  connect (ui_->backButton,
           SIGNAL (clicked ()),
           this,
           SLOT (backButtonPressed ()));

  connect (ui_->nextButton,
           SIGNAL (clicked ()),
           this,
           SLOT (nextButtonPressed ()));

  // Connect file controls
  connect (ui_->selectFolderButton,
           SIGNAL (clicked ()),
           this,
           SLOT (selectFolderButtonPressed ()));

  connect (ui_->selectFilesButton,
           SIGNAL (clicked ()),
           this,
           SLOT (selectFilesButtonPressed ()));

  connect (ui_->fullscreenButton,
           SIGNAL (clicked ()),
           this,
           SLOT (toggleFullscreen ()));

  connect (ui_->snapshotButton,
           SIGNAL (clicked ()),
           this,
           SLOT (snapshotButtonPressed ()));

  connect (ui_->screenshotButton,
           SIGNAL (clicked ()),
           this,
           SLOT (screenshotButtonPressed ()));

  connect (ui_->indexSlider,
           SIGNAL (valueChanged (int)),
           this,
           SLOT (indexSliderValueChanged (int)));
}


void PCDVideoPlayer::backButtonPressed () {
  if (current_frame_ == 0) // Already in the beginning
  {
    PCL_DEBUG ("[PCDVideoPlayer::nextButtonPressed] : reached the end\n");
    // reset to end
    current_frame_ = static_cast <unsigned int> (nr_of_frames_ - 1);
  } else {
    current_frame_--;
    cloud_modified_ = true;
    // Update slider position
    ui_->indexSlider->setSliderPosition (current_frame_);
  }
}


void PCDVideoPlayer::nextButtonPressed () {
  if (current_frame_ == (nr_of_frames_ - 1)) // Reached the end
  {
    PCL_DEBUG ("[PCDVideoPlayer::nextButtonPressed] : reached the end\n");
    // Reset to beginning
    current_frame_ = 0;
  } else {
    current_frame_++;
    cloud_modified_ = true;
    // Update slider position
    ui_->indexSlider->setSliderPosition (current_frame_);
  }
}


void PCDVideoPlayer::selectFolderButtonPressed () {
  pcd_files_.clear ();     // Clear the std::vector
  pcd_paths_.clear ();     // Clear the boost filesystem paths

  dir_ = QFileDialog::getExistingDirectory (
      this,
      tr ("Open Directory"),
      QDir::currentPath (),
      QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
  );

  boost::filesystem::directory_iterator end_itr;

  if (boost::filesystem::is_directory (dir_.toStdString ())) {
    for (boost::filesystem::directory_iterator itr (dir_.toStdString ());
         itr != end_itr; ++itr) {
      std::string ext = itr->path ().extension ().string ();
      if (ext.compare (".pcd") == 0) {
        pcd_files_.push_back (itr->path ().string ());
        pcd_paths_.push_back (itr->path ());
      } else {
        // Found non pcd file
        PCL_DEBUG ("[PCDVideoPlayer::selectFolderButtonPressed] : "
                       "found a different file\n");
      }
    }
    std::sort (begin (pcd_files_), end (pcd_files_));
    std::sort (begin (pcd_paths_), end (pcd_paths_));
  } else {
    PCL_ERROR("Path is not a directory\n");
    exit (-1);
  }
  nr_of_frames_ = pcd_files_.size ();
  PCL_DEBUG ("[PCDVideoPlayer::selectFolderButtonPressed] : found %d files\n",
             nr_of_frames_);

  if (nr_of_frames_ == 0) {
    PCL_ERROR ("Please select valid pcd folder\n");
    cloud_present_ = false;
    return;
  } else {
    // Reset the Slider
    // Set cursor back in the beginning
    ui_->indexSlider->setValue (0);
    // Rescale the slider
    ui_->indexSlider->setRange (0, static_cast <int> (nr_of_frames_ - 1));

    current_frame_ = 0;

    cloud_present_ = true;
    cloud_modified_ = true;
    snapshot_dir_ =
        boost::filesystem::path {dir_.toStdString ().c_str ()} / "snapshots";
    screenshot_dir_ =
        boost::filesystem::path {dir_.toStdString ().c_str ()} / "screenshots";
  }
}


void PCDVideoPlayer::selectFilesButtonPressed () {
  pcd_files_.clear ();  // Clear the std::vector
  pcd_paths_.clear ();     // Clear the boost filesystem paths

  auto qt_pcd_files = QFileDialog::getOpenFileNames (
      this,
      "Select one or more PCD files to open",
      "/home",
      "PointClouds (*.pcd)"
  );
  nr_of_frames_ = static_cast <unsigned long> (qt_pcd_files.size ());
  PCL_INFO ("[PCDVideoPlayer::selectFilesButtonPressed] : selected %ld files\n",
            nr_of_frames_);

  if (nr_of_frames_ == 0) {
    PCL_ERROR ("Please select valid pcd files\n");
    cloud_present_ = false;
    return;
  } else {
    for (int i = 0; i < qt_pcd_files.size (); i++) {
      pcd_files_.push_back (qt_pcd_files.at (i).toStdString ());
    }

    current_frame_ = 0;

    // Reset the Slider
    // set cursor back in the beginning
    ui_->indexSlider->setValue (0);
    // rescale the slider
    ui_->indexSlider->setRange (0, static_cast <int> (nr_of_frames_ - 1));

    cloud_present_ = true;
    cloud_modified_ = true;
  }
}


void PCDVideoPlayer::timeoutSlot () {
  if (play_mode_) {
    if (speed_counter_ == speed_value_) {
      // Reached the end
      if (current_frame_ == (nr_of_frames_ - 1)) {
        // reset to beginning
        current_frame_ = 0;
      } else {
        current_frame_++;
        cloud_modified_ = true;
        // Update the slider position
        ui_->indexSlider->setSliderPosition (current_frame_);
      }
    } else {
      speed_counter_++;
    }
  }

  if (cloud_present_ && cloud_modified_) {
    if (pcl::io::loadPCDFile <pcl::PointXYZRGBA> (pcd_files_[current_frame_],
                                                  *cloud_) == -1) {
      PCL_ERROR ("[PCDVideoPlayer::timeoutSlot] : Couldn't read file %s\n");
    }

    if (!vis_->updatePointCloud (cloud_, "cloud_")) {
      vis_->addPointCloud (cloud_, "cloud_");
      //      vis_->resetCameraViewpoint("cloud_");

      if (!this->camera_init) {
        // Fix camera position
//        vis_->initCameraParameters ();
        vis_->resetCameraViewpoint ("cloud_");
        vis_->setCameraPosition (0, 0, 0,
                                 0, 0, 1,
                                 0, -1, 0);
//        pcl::visualization::Camera camera;
//        vis_->getCameraParameters (camera);
//        camera.view[1] *= -1;
//        camera.window_size[0] = 640;
//        camera.window_size[1] = 480;
//        camera.fovy =
//        vis_->setCameraPosition (0.0, 0.0, 0.0,
//                                 0.0, 0.0, 1.0,
//                                 0.0, -1.0, 0.0);
//        vis_->setCameraParameters (camera);
        this->camera_init != this->camera_init;
      }
    }
    cloud_modified_ = false;
  }
  ui_->qvtkWidget->update ();
}


void PCDVideoPlayer::indexSliderValueChanged (int value) {
  PCL_DEBUG ("[PCDVideoPlayer::indexSliderValueChanged] : (I) : value %d\n",
             value);
  current_frame_ = value;
  cloud_modified_ = true;
}


void PCDVideoPlayer::toggleFullscreen () {
  if (!this->ui_->qvtkWidget->isFullScreen ()) {
    this->showFullScreen ();
    this->ui_->qvtkWidget->showFullScreen ();

  }
  //  else
  //  else
  //    this->ui_->qvtkWidget->showFullScreen(false);
}


void PCDVideoPlayer::snapshotButtonPressed () {

  namespace fs = boost::filesystem;
  if (pcd_paths_.size () == 0) {
    PCL_DEBUG (
        "[PCDVideoPlayer::snapshotButtonPressed] : "
            "No pcd files have been loaded\n");
    return;
  }

  if (!fs::exists (snapshot_dir_) || !fs::is_directory (snapshot_dir_)) {
    if (!fs::create_directory (snapshot_dir_)) {
      PCL_DEBUG (
          "[PCDVideoPlayer::snapshotButtonPressed] : "
              "Unable to create snapshot directory\n");
      return;
    }
  }

  // Get current pcd file and copy to snapshot directory
  if (current_frame_ > 0 && current_frame_ < pcd_paths_.size ()) {
    auto & current_file = pcd_paths_.at (current_frame_);
    if (!fs::exists (current_file) || !fs::is_regular_file (current_file)) {
      PCL_DEBUG (
          "[PCDVideoPlayer::snapshotButtonPressed] : "
              "Current file does not exist\n");
      return;
    }
    auto destination_file = snapshot_dir_ / current_file.filename ();
    fs::copy_file (current_file,
                   destination_file,
                   fs::copy_option::overwrite_if_exists);
  }
}


void PCDVideoPlayer::screenshotButtonPressed () {
  namespace fs = boost::filesystem;
  if (pcd_paths_.size () == 0) {
    PCL_DEBUG (
        "[PCDVideoPlayer::screenshotButtonPressed] : "
            "No pcd files have been loaded\n");
    return;
  }

  if (!fs::exists (screenshot_dir_) || !fs::is_directory (screenshot_dir_)) {
    if (!fs::create_directory (screenshot_dir_)) {
      PCL_DEBUG (
          "[PCDVideoPlayer::screenshotButtonPressed] : "
              "Unable to create screenshot directory\n");
      return;
    }
  }

  // Get a screenshot of the current view and save it to screenshot directory.
  // Get current pcd file and copy to screenshot directory
  if (current_frame_ > 0 && current_frame_ < pcd_paths_.size ()) {
    auto & current_file = pcd_paths_.at (current_frame_);
    if (!fs::exists (current_file) || !fs::is_regular_file (current_file)) {
      PCL_DEBUG (
          "[PCDVideoPlayer::screenshotButtonPressed] : "
              "Current file does not exist\n");
      return;
    }

    // Get the filename
    auto destination_file = screenshot_dir_ / "screnshot.png";
    std::cout << "Saving " << destination_file.string () << std::endl;
    vis_->saveScreenshot (destination_file.string ());
  }
}


void print_usage () {
  PCL_INFO ("PCDVideoPlayer V0.1\n");
  PCL_INFO ("-------------------\n");
  PCL_INFO ("\tThe slider accepts focus on Tab and provides both a mouse wheel "
                "and a keyboard interface. The keyboard interface is the "
                "following:\n");
  PCL_INFO ("\t  Left/Right move a horizontal slider by one single step.\n");
  PCL_INFO ("\t  Up/Down move a vertical slider by one single step.\n");
  PCL_INFO ("\t  PageUp moves up one page.\n");
  PCL_INFO ("\t  PageDown moves down one page.\n");
  PCL_INFO ("\t  Home moves to the start (mininum).\n");
  PCL_INFO ("\t  End moves to the end (maximum).\n");
  PCL_INFO ("\t  F toggles fullscreen mode.\n");
}


int main (int argc, char ** argv) {
  QApplication app (argc, argv);

  PCDVideoPlayer VideoPlayer;

  VideoPlayer.show ();

  return (app.exec ());
}
