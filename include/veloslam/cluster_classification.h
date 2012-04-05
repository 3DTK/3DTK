#pragma once
#include "veloslam/veloscan.h"
#include "veloslam/svm.h"

class Cluster_Classification
{
public:
	Cluster_Classification(void);
	~Cluster_Classification(void);

public:
    svm_model *bus_classifier;
    string *bus_classifier_filename;
    svm_node *feature_nod;
};




