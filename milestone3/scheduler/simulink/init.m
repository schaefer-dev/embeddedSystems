def = legacy_code('initialize')

def.SourceFiles = {'matlab.c'}
def.HeaderFiles = {'matlab.h'}

def.SFunctionName = 'CAdd'
def.OutputFcnSpec = 'double y1 = add(double u1, double u2)';
legacy_code('sfcn_cmex_generate',def)
legacy_code('compile',def)
%legacy_code('sfcn_tlc_generate',def)
