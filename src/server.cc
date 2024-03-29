// Copyright (c) 2018, 2019, 2020 CNRS and Airbus S.A.S
// Author: Joseph Mirabel
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following
// disclaimer in the documentation and/or other materials provided
// with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include "server.hh"

#include <hpp/util/exception.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/corbaserver/server.hh>
#include <hpp/corbaserver/servant-base.hh>
#include <hpp/manipulation/problem-solver.hh>

#include "hpp/agimus_idl/discretization.hh"
#include <hpp/agimus/discretization.hh>

#include "hpp/agimus_idl/point-cloud.hh"
#include <hpp/agimus/point-cloud.hh>

namespace hpp {
  namespace agimus {
    namespace impl {
      agimus_idl::Discretization_ptr Server::getDiscretization ()
      {
        discretization_ =
          Discretization::create (server_->problemSolver()->robot());

        agimus_impl::Discretization* servant =
          new agimus_impl::Discretization (server_->parent(),
              discretization_);
        servant->persistantStorage(false);

        return corbaServer::makeServant<agimus_idl::Discretization_ptr>
          (server_->parent(), servant);
      }

      agimus_idl::PointCloud_ptr Server::getPointCloud ()
      {
	manipulation::ProblemSolverPtr_t ps
	  (dynamic_cast<manipulation::ProblemSolverPtr_t>
	   (server_->problemSolver()));
	if (!ps){
	  throw std::invalid_argument("ProblemSolver instance is not of type "
				      "hpp::manipulation::ProblemSolver.");
	}
        pointCloud_ =
          PointCloud::create (ps);

        agimus_impl::PointCloud* servant =
          new agimus_impl::PointCloud (server_->parent(), pointCloud_);
        servant->persistantStorage(false);

        return corbaServer::makeServant<agimus_idl::PointCloud_ptr>
          (server_->parent(), servant);
      }

    } // namespace impl

    ServerPlugin::ServerPlugin (corbaServer::Server* server)
      : corbaServer::ServerPlugin (server),
      serverImpl_ (NULL)
    {}

    ServerPlugin::~ServerPlugin ()
    {
      if (serverImpl_  ) delete serverImpl_;
    }

    std::string ServerPlugin::name () const
    {
      return "agimus";
    }

    /// Start corba server
    void ServerPlugin::startCorbaServer(const std::string& contextId,
				  const std::string& contextKind)
    {
      initializeTplServer (serverImpl_, contextId, contextKind, name(), "server");
      serverImpl_->implementation ().setServer (this);
    }

    ::CORBA::Object_ptr ServerPlugin::servant(const std::string& name) const
    {
      if (name == "server") return serverImpl_->implementation()._this();
      throw std::invalid_argument ("No servant " + name);
    }
  } // namespace agimus
} // namespace hpp

HPP_CORBASERVER_DEFINE_PLUGIN(hpp::agimus::ServerPlugin)
