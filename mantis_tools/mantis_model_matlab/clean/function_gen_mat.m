function [  ] = function_gen_mat( m, m_name )
%FUNCTION_GEN_MAT Generates a C++ function to calculate matrices 
%   Detailed explanation goes here
    
    var_str = symvar(m);

    %Generate function call
    fun_str = ['void calc_', m_name, '(Eigen::Vector3d& m'];
    for i = 1:numel(var_str)
        fun_str = [fun_str, ', ', char(var_str(i))];
    end
    fun_str = [fun_str, ') {', newline];
    
    %Generate function code
    fun_str = [fun_str, ccode(m)];
    
    %Close function call
    fun_str = [fun_str, newline, '}'];

    %Clean
    fun_str = regexprep(fun_str, '  ', '\t');


    %Save function to disk
    filename = ['./gen_code/calc_', m_name, '.cpp'];
    fid = fopen(filename, 'w');

    if fid ~= -1
        fprintf(fid, fun_str);
        fclose(fid);
    else
        error(['Could not create file: ', filename]);
    end
end

