import pyrbdl as rbdl
print(dir(rbdl))

import numpy as np

model = rbdl.Model()

body_a = rbdl.Body (1., [1., 0., 0.], [1., 1., 1.])
joint_a = rbdl.Joint( [0., 0., 1., 0., 0., 0.])
body_a_id = model.AddBody(0, rbdl.Xtrans([0., 0., 0.]), joint_a, body_a, "body_a");

print("model.dof_count=", model.dof_count)