/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2024-12-15 00:33:33
 * @LastEditTime: 2024-12-16 00:31:52
 * @FilePath: /dive-into-contingency-planning/src/simulator/simulator_base.cpp
 * Copyright 2024 puyu, All Rights Reserved.
 */

#include "simulator/simulator_base.hpp"

void SimulatorBase::check10hz(std::chrono::steady_clock::time_point tbegin) {
    auto tend = std::chrono::steady_clock::now();
    auto used = std::chrono::duration_cast<std::chrono::milliseconds>(tend - tbegin);
    if (used.count() > 100) {
        return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100 - used.count()));
}

void SimulatorBase::imshow(const Outlook& img, const std::vector<double>& state) {
    static PyObject* imshow_func = nullptr;
    if (imshow_func == nullptr) {
        Py_Initialize();
        _import_array();

        std::filesystem::path source_file_path(__FILE__);
        std::filesystem::path project_path = source_file_path.parent_path().parent_path().parent_path();
        std::string script_path = project_path / "images" / "materials";
        PyRun_SimpleString("import sys");
        PyRun_SimpleString(fmt::format("sys.path.append('{}')", script_path).c_str());

        PyObject* py_name = PyUnicode_DecodeFSDefault("imshow");
        PyObject* py_module = PyImport_Import(py_name);
        Py_DECREF(py_name);
        if (py_module != nullptr) {
            imshow_func = PyObject_GetAttrString(py_module, "imshow");
        }
        if (imshow_func == nullptr || !PyCallable_Check(imshow_func)) {
            SPDLOG_ERROR("imshow(): py.imshow call failed !");
            imshow_func = nullptr;
        }
    }

    PyObject* vehicle_state = matplotlibcpp::detail::get_array(state);
    npy_intp dims[3] = {img.rows, img.cols, img.colors};

    const float* imptr = &(img.data[0]);

    PyObject* args = PyTuple_New(4);
    PyTuple_SetItem(args, 0, PyArray_SimpleNewFromData(3, dims, NPY_FLOAT, (void*)imptr));
    PyTuple_SetItem(args, 1, vehicle_state);
    PyTuple_SetItem(args, 2, PyFloat_FromDouble(img.visual_width));
    PyTuple_SetItem(args, 3, PyFloat_FromDouble(img.visual_height));

    PyObject* ret = PyObject_CallObject(imshow_func, args);

    Py_DECREF(args);
    if (ret) {
        Py_DECREF(ret);
    }
}
