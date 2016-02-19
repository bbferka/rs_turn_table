/*
 * Copyright (c) 2011, Ferenc Balint-Benczedi <balintbe@tzi.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Intelligent Autonomous Systems Group/
 *       Technische Universitaet Muenchen nor the names of its contributors
 *       may be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "uima/api.hpp"


#include <ros/ros.h>
#include <std_msgs/String.h>


#include <rs/scene_cas.h>
#include <rs/utils/exception.h>
#include <rs/utils/output.h>

#include <scanning_table_msgs/scanning_tableAction.h>
#include <actionlib/client/simple_action_client.h>


using namespace uima;



class MoveTurnTable: public Annotator
{
  typedef actionlib::SimpleActionClient<scanning_table_msgs::scanning_tableAction> ActionClient;
private:
  float angularResolution;
  float currentPosition;
  ros::NodeHandle nh;
  ActionClient *aClient;
public:

  MoveTurnTable(): angularResolution(2.0), currentPosition(0.0), nh("~")
  {
    aClient = new ActionClient("/scanning_table_action_server", true);
    outInfo("Waiting for Turn Table Server");
    aClient->waitForServer();
    outInfo("Server Found..");
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    ctx.extractValue("angularResolution", angularResolution);
    return (TyErrorId) UIMA_ERR_NONE;
  }

  TyErrorId typeSystemInit(TypeSystem const &type_system)
  {
    return (TyErrorId) UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {

    return (TyErrorId) UIMA_ERR_NONE;
  }


  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    // declare variables for kinect data
    outInfo("process start");
    if(currentPosition < 360)
    {
      outInfo("Angle Command: " << angularResolution);
      scanning_table_msgs::scanning_tableActionGoal goal;
      goal.goal.angle = currentPosition * M_PI / 180;
      goal.goal.release_brake_timeout = 20.0;
      aClient->sendGoal(goal.goal);
      aClient->waitForResult(ros::Duration(20.0));
      if(aClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        outInfo("The turn_table moved to " << currentPosition);
      }
      currentPosition += angularResolution;
      outInfo("increased position to: " << currentPosition);
    }
    else
    {
      ros::shutdown();
    }
    usleep(100000);
    return (TyErrorId) UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(MoveTurnTable)

