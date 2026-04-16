#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "underwater_vehicle_model.hpp"

namespace py = pybind11;

PYBIND11_MODULE(mvm_py, m) {
    m.doc() = "Python bindings for the UnderwaterVehicleModel library";

    py::class_<mvm::UnderwaterVehicleModel>(m, "UnderwaterVehicleModel")
        .def(py::init<const std::string&, const std::string&>(),
             py::arg("config_path"),
             py::arg("model_name"),
             "Create a model from a libconfig file path and model name.")
        .def("update_model", &mvm::UnderwaterVehicleModel::UpdateModel,
             py::arg("velocity"),
             py::arg("pose"),
             "Update dynamics matrices given current velocity and pose.")
        .def("compute_acceleration", &mvm::UnderwaterVehicleModel::ComputeAcceleration,
             py::arg("forces"),
             "Compute acceleration from thruster forces.")
        .def_property_readonly("num_thrusters", &mvm::UnderwaterVehicleModel::GetNumThrusters,
             "Number of thrusters.")
        .def_property_readonly("thrusters_wrench_matrix",
             &mvm::UnderwaterVehicleModel::GetThrustersWrenchMatrix,
             py::return_value_policy::reference_internal,
             "Thruster wrench matrix (6 x N).")
        .def_property_readonly("mass_matrix", &mvm::UnderwaterVehicleModel::GetMassMatrix,
             py::return_value_policy::reference_internal,
             "Mass matrix (6 x 6).")
        .def_property_readonly("coriolis_matrix", &mvm::UnderwaterVehicleModel::GetCoriolisMatrix,
             py::return_value_policy::reference_internal,
             "Coriolis matrix (6 x 6).")
        .def_property_readonly("damping_matrix", &mvm::UnderwaterVehicleModel::GetDampingMatrix,
             py::return_value_policy::reference_internal,
             "Damping matrix (6 x 6).")
        .def_property_readonly("restoring_forces", &mvm::UnderwaterVehicleModel::GetRestoringForces,
             py::return_value_policy::reference_internal,
             "Restoring forces (6).");
}
