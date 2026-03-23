/**
 * ----------------------------------------------------------------------------
 * Company    : SLAC National Accelerator Laboratory
 * ----------------------------------------------------------------------------
 * Description:
 * Python module setup for rogue.protocols.rocev2
 * ----------------------------------------------------------------------------
 **/
#include "rogue/Directives.h"

#include "rogue/protocols/rocev2/module.h"

#include <boost/python.hpp>

#include "rogue/protocols/rocev2/Core.h"
#include "rogue/protocols/rocev2/Server.h"

namespace bp  = boost::python;
namespace rpr = rogue::protocols::rocev2;

void rpr::setup_module() {
  // Map to rogue.protocols.rocev2 sub-module
  bp::object module(
                    bp::handle<>(bp::borrowed(PyImport_AddModule("rogue.protocols.rocev2"))));

  bp::scope().attr("rocev2") = module;
  bp::scope io_scope = module;

  rpr::Core::setup_python();
  rpr::Server::setup_python();
}
