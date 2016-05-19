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
 * Copyright 2015 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_SUPPORT_PROTO_ARMADILLO_H
#define UTILITY_SUPPORT_PROTO_ARMADILLO_H

#include <armadillo>
#include "message/Vector.pb.h"
#include "message/Matrix.pb.h"

message::vec2&  operator<< ( message::vec2& proto,  const arma::vec2& vec);
message::vec3&  operator<< ( message::vec3& proto,  const arma::vec3& vec);
message::vec4&  operator<< ( message::vec4& proto,  const arma::vec4& vec);
message::fvec2& operator<< (message::fvec2& proto, const arma::fvec2& vec);
message::fvec3& operator<< (message::fvec3& proto, const arma::fvec3& vec);
message::fvec4& operator<< (message::fvec4& proto, const arma::fvec4& vec);
message::ivec2& operator<< (message::ivec2& proto, const arma::ivec2& vec);
message::ivec3& operator<< (message::ivec3& proto, const arma::ivec3& vec);
message::ivec4& operator<< (message::ivec4& proto, const arma::ivec4& vec);
message::uvec2& operator<< (message::uvec2& proto, const arma::uvec2& vec);
message::uvec3& operator<< (message::uvec3& proto, const arma::uvec3& vec);
message::uvec4& operator<< (message::uvec4& proto, const arma::uvec4& vec);

message::vec2&  operator<< ( message::vec2& proto,  const arma::vec& vec);
message::vec3&  operator<< ( message::vec3& proto,  const arma::vec& vec);
message::vec4&  operator<< ( message::vec4& proto,  const arma::vec& vec);
message::fvec2& operator<< (message::fvec2& proto, const arma::fvec& vec);
message::fvec3& operator<< (message::fvec3& proto, const arma::fvec& vec);
message::fvec4& operator<< (message::fvec4& proto, const arma::fvec& vec);
message::ivec2& operator<< (message::ivec2& proto, const arma::ivec& vec);
message::ivec3& operator<< (message::ivec3& proto, const arma::ivec& vec);
message::ivec4& operator<< (message::ivec4& proto, const arma::ivec& vec);
message::uvec2& operator<< (message::uvec2& proto, const arma::uvec& vec);
message::uvec3& operator<< (message::uvec3& proto, const arma::uvec& vec);
message::uvec4& operator<< (message::uvec4& proto, const arma::uvec& vec);

message::mat22&  operator<< ( message::mat22& proto,  const arma::mat22& mat);
message::mat33&  operator<< ( message::mat33& proto,  const arma::mat33& mat);
message::mat44&  operator<< ( message::mat44& proto,  const arma::mat44& mat);
message::fmat22& operator<< (message::fmat22& proto, const arma::fmat22& mat);
message::fmat33& operator<< (message::fmat33& proto, const arma::fmat33& mat);
message::fmat44& operator<< (message::fmat44& proto, const arma::fmat44& mat);
message::imat22& operator<< (message::imat22& proto, const arma::imat22& mat);
message::imat33& operator<< (message::imat33& proto, const arma::imat33& mat);
message::imat44& operator<< (message::imat44& proto, const arma::imat44& mat);
message::umat22& operator<< (message::umat22& proto, const arma::umat22& mat);
message::umat33& operator<< (message::umat33& proto, const arma::umat33& mat);
message::umat44& operator<< (message::umat44& proto, const arma::umat44& mat);

 message::mat22& operator<< ( message::mat22& proto,  const arma::mat& mat);
 message::mat33& operator<< ( message::mat33& proto,  const arma::mat& mat);
 message::mat44& operator<< ( message::mat44& proto,  const arma::mat& mat);
message::fmat22& operator<< (message::fmat22& proto, const arma::fmat& mat);
message::fmat33& operator<< (message::fmat33& proto, const arma::fmat& mat);
message::fmat44& operator<< (message::fmat44& proto, const arma::fmat& mat);
message::imat22& operator<< (message::imat22& proto, const arma::imat& mat);
message::imat33& operator<< (message::imat33& proto, const arma::imat& mat);
message::imat44& operator<< (message::imat44& proto, const arma::imat& mat);
message::umat22& operator<< (message::umat22& proto, const arma::umat& mat);
message::umat33& operator<< (message::umat33& proto, const arma::umat& mat);
message::umat44& operator<< (message::umat44& proto, const arma::umat& mat);

#endif
