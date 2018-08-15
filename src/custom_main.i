%module custom_main
%{
/* Put header files here or function declarations like below */

#define SWIG_FILE_WITH_INIT



#include "custom_main.h"


%}





%typemap(in) (int argc, char *argv[]) {
int i;
if (!PyList_Check($input)) {
PyErr_SetString(PyExc_ValueError, "Expecting a list");
return NULL;
}
$1 = PyList_Size($input);
$2 = (char **) malloc(($1+1)*sizeof(char *));
for (i = 0; i < $1; i++) {
PyObject *s = PyList_GetItem($input,i);
if (!PyString_Check(s)) {
    free($2);
    PyErr_SetString(PyExc_ValueError, "List items must be strings");
    return NULL;
}
$2[i] = PyString_AsString(s);
}
$2[i] = 0;
}

%typemap(freearg) (int argc, char *argv[]) {
free($2); // If here is uneeded, free(NULL) is legal
}


%include "numpy.i"

%init %{
import_array();
%}

%apply (double* IN_ARRAY1, int DIM1) {(double* seq, int n)}

%include "std_string.i" 
%include "custom_main.h"
