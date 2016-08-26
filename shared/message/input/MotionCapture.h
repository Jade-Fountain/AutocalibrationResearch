/*
 * This file is part of the Autocalibration Codebase.
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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGES_INPUT_SERVOID_H
#define MESSAGES_INPUT_SERVOID_H

#include <string>
#include "utility/math/matrix/Transform3D.h"

namespace message {
    namespace input {

        struct RigidBodyFrame{
            std::map<int, utility::math::matrix::Transform3D> poses;
        };

        struct OpenNIData{
            std::map<int, RigidBodyFrame> users;
        };

        struct MotionCapture {

            struct Marker {
                uint32_t id;
                arma::fvec3 position;
                float size;
            };

            struct MarkerSet {
                std::string name;
                std::vector<Marker> markers;
            };

            struct RigidBody {
                uint32_t id;
                arma::fvec3 position;
                arma::fvec4 rotation;
                std::vector<Marker> markers;
                float error;
                bool trackingValid;

                // Information added by the model
                std::string name;
                arma::fvec3 offset;
                RigidBody* parent;
                std::vector<RigidBody*> children;
            };

            struct Skeleton {
                uint32_t id;
                std::vector<RigidBody> bones;

                // Information added by the model
                std::string name;
            };

            struct LabeledMarker : public Marker {
                bool occluded;
                bool pointCloudSolved;
                bool modelSolved;
            };

            struct ForcePlate {
                uint32_t id;
                std::vector<std::vector<float>> channels;
            };

            uint32_t frameNumber;
            std::vector<MarkerSet> markerSets;
            std::vector<Marker> markers;
            std::vector<RigidBody> rigidBodies;
            std::vector<Skeleton> skeletons;
            std::vector<LabeledMarker> labeledMarkers;
            std::vector<ForcePlate> forcePlates;

            float latency;
            uint32_t timecode;
            uint32_t timecodeSub;
            double timestamp;
            bool recording;
            bool trackedModelsChanged;
        };
    }
}

#endif
