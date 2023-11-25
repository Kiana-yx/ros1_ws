#!/usr/bin/env python

# import package
from dynamic_reconfigure.parameter_generator_catkin import *

# create parameter generator
gen = ParameterGenerator()

# add parameters into parameter generator
gen.add("int_param", int_t, 0, "int type", 0, 0, 100)
gen.add("double_param", double_t, 1, "double type", 1.57, 0, 3.14)
gen.add("bool_param", bool_t, 2, "bool type", True)
measure = gen.enum([gen.const("small", int_t, 0, "small"),
                    gen.const("medium", int_t, 1, "medium"),
                    gen.const("big", int_t, 2, "big")], "choice")
gen.add("list_param", int_t, 3, "alternative choices",
        0, 0, 2, edit_method=measure)   #edit_method：用于接收enum函数返回的句柄；

# generate intermediate file
exit(gen.generate("demo01", "config", "dr1"))
