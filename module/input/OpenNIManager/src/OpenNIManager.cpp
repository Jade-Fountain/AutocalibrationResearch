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
#include "message/input/OpenNIImage.h"
#include "message/input/MotionCapture.h"

#include "utility/math/matrix/Transform3D.h"
#include "utility/math/geometry/UnitQuaternion.h"

namespace module {
namespace input {

    using message::input::OpenNIImage;
    using message::input::OpenNIData;
    using message::input::RigidBodyFrame;

    using utility::math::matrix::Transform3D;
    using utility::math::geometry::UnitQuaternion;

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

            excludedJoints.insert(int(nite::JOINT_LEFT_HAND));
            excludedJoints.insert(int(nite::JOINT_RIGHT_HAND));
            excludedJoints.insert(int(nite::JOINT_RIGHT_FOOT));
            excludedJoints.insert(int(nite::JOINT_LEFT_FOOT));
            excludedJoints.insert(int(nite::JOINT_NECK));



    	});

		on<Every<30, Per<std::chrono::seconds>>, Single>().then("OpenNIManager Read loop",[this](){
        
            nite::UserTrackerFrameRef userTrackerFrame;
            auto mocap = std::make_unique<OpenNIData>();


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
                mocap->users[i] = RigidBodyFrame();
                const nite::UserData& user = users[i];
                updateUserState(user,userTrackerFrame.getTimestamp());
                if (user.isNew())
                {
                    userTracker.startSkeletonTracking(user.getId());
                }
                else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
                {
                    for(int j = 0; j < NITE_JOINT_COUNT; j++){
                        //Skip irrelevant joints
                        if(excludedJoints.count(j) > 0) continue;
                        auto& joint = user.getSkeleton().getJoint((nite::JointType)j);
                        UnitQuaternion q(joint.getOrientation().w,
                                         joint.getOrientation().x,
                                         joint.getOrientation().y,
                                         joint.getOrientation().z);
                        Transform3D pose(q);
                        
                        //Convert to m from mm
                        pose.translation() = 1e-3 * (arma::vec3{joint.getPosition().x,joint.getPosition().y,joint.getPosition().z});
                        
                        //Reflect to RHS OpenGL coordinate system
                        auto refl = Transform3D::createScale(arma::vec3{1,1,-1});
                        pose = refl * pose * refl;
                        // pose = Transform3D::createRotationY(M_PI) * pose;

                        mocap->users[i].poses[j] = pose;
                    }
                }
            }
            emit(mocap);

            //Depth frame output
            openni::VideoFrameRef depthFrame;
            depthFrame = userTrackerFrame.getDepthFrame();

            // if (m_pTexMap == NULL)
            // {
            //     // Texture map init
            //     m_nTexMapX = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionX(), TEXTURE_SIZE);
            //     m_nTexMapY = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionY(), TEXTURE_SIZE);
            //     m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];
            // }

            const nite::UserMap& userLabels = userTrackerFrame.getUserMap();

            if (!depthFrame.isValid())
            {
                return;
            }

            auto texOut = std::make_unique<OpenNIImage>(depthFrame.getWidth(),depthFrame.getHeight());

            // memset(m_pTexMap, 0, m_nTexMapX*m_nTexMapY*sizeof(openni::RGB888Pixel));

            float factor[3] = {1, 1, 1};
            // check if we need to draw depth frame to texture
            if (depthFrame.isValid())
            {
                const nite::UserId* pLabels = userLabels.getPixels();

                const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame.getData();
                openni::RGB888Pixel* pTexRow = texOut->data();// + depthFrame.getCropOriginY() * m_nTexMapX;
                int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

                for (int y = 0; y < depthFrame.getHeight(); ++y)
                {
                    const openni::DepthPixel* pDepth = pDepthRow;
                    openni::RGB888Pixel* pTex = pTexRow;// + depthFrame.getCropOriginX();

                    for (int x = 0; x < depthFrame.getWidth(); ++x, ++pDepth, ++pTex, ++pLabels)
                    {
                        if (*pDepth != 0)
                        {
                            if (*pLabels == 0)
                            {
                                // if (!g_drawBackground)
                                // {
                                //     factor[0] = factor[1] = factor[2] = 0;

                                // }
                                // else
                                // {
                                    factor[0] = Colors[colorCount][0];
                                    factor[1] = Colors[colorCount][1];
                                    factor[2] = Colors[colorCount][2];

                                // }

                            }
                            else
                            {
                                factor[0] = Colors[*pLabels % colorCount][0];
                                factor[1] = Colors[*pLabels % colorCount][1];
                                factor[2] = Colors[*pLabels % colorCount][2];

                            }
                            pTex->r = (*pDepth * 255 / 5000) * factor[0];
                            pTex->g = (*pDepth * 255 / 5000) * factor[1];
                            pTex->b = (*pDepth * 255 / 5000) * factor[2];

                            factor[0] = factor[1] = factor[2] = 1;
                        }
                    }

                    pDepthRow += rowSize;
                    pTexRow += rowSize;
                }
            }


            emit(texOut);
		});

        on<Trigger<Shutdown>>().then([this]{
            std::cout << "Shutting down openni..." << std::endl;
            nite::NiTE::shutdown();
        });
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
