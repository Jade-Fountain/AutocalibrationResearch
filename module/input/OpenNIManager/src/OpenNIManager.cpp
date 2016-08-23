/*
 * This file is part of Autocalibration Codebase.
 *
 * The Autocalibration Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Autocalibration Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Autocalibration Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2016 Autocalibration <nubots@nubots.net>
 */

#include "OpenNIManager.h"

namespace module {
namespace input {

    OpenNIManager::OpenNIManager(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment))
    {

    	on<Startup>().then("OpenNIManager Start",[this](){
	    	//Startup
            nite::Status niteRc;

            nite::NiTE::initialize();

            niteRc = userTracker.create();
            if (niteRc != nite::STATUS_OK)
            {
                log("Couldn't create user tracker\n", "Status:", niteRc);
                // powerplant.shutdown();
                return;
            }
            log("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");

    	});

		on<Every<30, Per<std::chrono::seconds>>, Single>().then("OpenNIManager Read loop",[this](){
        
            nite::UserTrackerFrameRef userTrackerFrame;

            nite::Status niteRc;
            niteRc = userTracker.readFrame(&userTrackerFrame);
            if (niteRc != nite::STATUS_OK)
            {
                log("Get next frame failed\n");
                return;
            }

            const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
            for (int i = 0; i < users.getSize(); ++i)
            {
                const nite::UserData& user = users[i];
                updateUserState(user,userTrackerFrame.getTimestamp());
                if (user.isNew())
                {
                    userTracker.startSkeletonTracking(user.getId());
                }
                else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
                {
                    const nite::SkeletonJoint& head = user.getSkeleton().getJoint(nite::JOINT_HEAD);
                    if (head.getPositionConfidence() > .5)
                        log("User", user.getId(), "(",head.getPosition().x, head.getPosition().y, head.getPosition().z, ")");
                }
            }


		});
    }
    
    OpenNIManager::~OpenNIManager(){
        nite::NiTE::shutdown();
    }

    void OpenNIManager::updateUserState(const nite::UserData& user, unsigned long long ts){
        if (user.isNew())
            userMessage("New", user, ts);
        else if (user.isVisible() && !g_visibleUsers[user.getId()])
            userMessage("Visible", user, ts);
        else if (!user.isVisible() && g_visibleUsers[user.getId()])
            userMessage("Out of Scene", user, ts);
        else if (user.isLost())
            userMessage("Lost", user, ts);

        g_visibleUsers[user.getId()] = user.isVisible();


        if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
        {
            switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
            {
            case nite::SKELETON_NONE:
                userMessage("Stopped tracking.", user, ts);
                break;
            case nite::SKELETON_CALIBRATING:
                userMessage("Calibrating...", user, ts);
                break;
            case nite::SKELETON_TRACKED:
                userMessage("Tracking!", user, ts);
                break;
            case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
            case nite::SKELETON_CALIBRATION_ERROR_HANDS:
            case nite::SKELETON_CALIBRATION_ERROR_LEGS:
            case nite::SKELETON_CALIBRATION_ERROR_HEAD:
            case nite::SKELETON_CALIBRATION_ERROR_TORSO:
                userMessage("Calibration Failed... :-|", user, ts);
                break;
            }
        }
    }

    void OpenNIManager::userMessage(std::string message, const nite::UserData& user,  unsigned long long ts){
        log("[", ts, "] User #", user.getId(), ":\t", message);
    }


} //input
} //module
