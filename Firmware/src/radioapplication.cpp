#include <radioapplication.h>

RadioApplication::RadioApplication() {
    websrv.addHandler(this);
}

RadioApplication::~RadioApplication() {
    websrv.removeHandler(this);
}