control_constrained = 100

term1 = 0.000000008062039184679460000000 * control_constrained ** 6 
term2 = - 0.000003544816635170370000000000 * control_constrained ** 5 
term3 = 0.000508078928071143000000000000 * control_constrained ** 4
term4 = - 0.031065721124832600000000000000 * control_constrained ** 3 
term5 = 0.874155641722609000000000000000 * control_constrained ** 2 
term6 = - 0.380647144396789000000000000000 * control_constrained 
term7 = 0.572951247857418000000000000000

thrust = term1 + term2 + term3 + term4 + term5 + term6 + term7

print(thrust)