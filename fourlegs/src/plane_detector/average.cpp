#include "../../include/plane_detector.h"

Pdetect::Plane Pdetect::ave(std::vector<Plane> &a){

    float a_sum=0.;
    float b_sum=0.;
    float c_sum=0.;
    float d_sum=0.;

    for(auto& elem : a)
    {
        a_sum += elem.a;
        b_sum += elem.b;
        c_sum += elem.c;
        d_sum += elem.d;
    }    

    Plane temp_avg;
    temp_avg.a= a_sum/a.size();
    temp_avg.b= b_sum/a.size();
    temp_avg.c= c_sum/a.size();
    temp_avg.d= d_sum/a.size();
    return temp_avg;
}

float Pdetect::m_var(std::vector<Plane> &a,Plane average){
    float ave_d=0.;
    float var_d=0.;
    float d_var_sum=0.;
    ave_d=average.d;
    for(auto& elem : a)
    {
        d_var_sum += pow((elem.d-ave_d),2.0);
    }

    var_d= d_var_sum/a.size();    
    //cout<<"var_d : "<<var_d<<endl;
    return var_d;
}

Matrix3f Pdetect::m_covar(std::vector<Plane> &a,Plane average){
    int rowsize = a.size();
    MatrixXf deviation(rowsize,3);

    for(auto& elem : a)
    {
        int i = &elem - &a[0];
        deviation(i,0)=elem.a - average.a;
        deviation(i,1)=elem.b - average.b;
        deviation(i,2)=elem.c - average.c;
    }
    //cout<<deviation<<endl;
    Matrix3f cov;
    cov = deviation.transpose() * deviation ;
    
    return cov;
}

Matrix3f Pdetect::e_covar(std::vector<Step> &a,Plane average){
    int rowsize = a.size();
    MatrixXf deviation(rowsize,3);

    for(auto& elem : a)
    {
        int i = &elem - &a[0];
        deviation(i,0)=elem.measurement.a - average.a;
        deviation(i,1)=elem.measurement.b - average.b;
        deviation(i,2)=elem.measurement.c - average.c;
    }
    //cout<<deviation<<endl;
    Matrix3f cov;
    cov = deviation.transpose() * deviation ;
    
    return cov;
}

Pdetect::Plane Pdetect::esti(std::vector<Step> &a){

    float a_sum=0.;
    float b_sum=0.;
    float c_sum=0.;
    float prior=0.;
    bool priorexist=false;
    float difference_sum=0.;
    float difference;

    for(auto& elem : a)
    {
        a_sum += elem.measurement.a;
        b_sum += elem.measurement.b;
        c_sum += elem.measurement.c;
        if (priorexist ==false){
            prior = elem.measurement.d;
            priorexist = true;
        }
        else if (priorexist == true){
            difference=(prior-elem.measurement.d);
            difference_sum += difference;
            prior= elem.measurement.d;
        }
    }    

    Plane temp_avg;
    temp_avg.a= a_sum/a.size();
    temp_avg.b= b_sum/a.size();
    temp_avg.c= c_sum/a.size();
    temp_avg.d= (difference_sum/(a.size()-1));

    float d_var_sum=0.;
    float var_d=0.;
    priorexist=false;

    for(auto& elem : a)
    {
        if (priorexist ==false){
            prior = elem.measurement.d;
            priorexist = true;
        }
        else if (priorexist == true){
            difference=(prior-elem.measurement.d);
            cout<<"difference : "<<difference<<endl;
            d_var_sum += pow((difference-temp_avg.d),2.0);
            cout<<"d-m square sum : "<<d_var_sum<<endl;
            prior= elem.measurement.d;
        }
    }
    var_d= d_var_sum/(a.size()-1);
    temp_avg.var_d= var_d;

    return temp_avg;
}


Pdetect::Plane Pdetect::esti_withkey(std::vector<Step> &a){

    float a_sum=0.;
    float b_sum=0.;
    float c_sum=0.;
    float prior=0.;
    bool priorexist=false;
    float difference_sum=0.;
    float difference;

    for(auto& elem : a)
    {
        a_sum += elem.key_stair.a;
        b_sum += elem.key_stair.b;
        c_sum += elem.key_stair.c;
        if (priorexist ==false){
            prior = elem.key_stair.d;
            priorexist = true;
        }
        else if (priorexist == true){
            difference=(prior-elem.key_stair.d);
            difference_sum += difference;
            prior= elem.key_stair.d;
        }
    }    

    Plane temp_avg;
    temp_avg.a= a_sum/a.size();
    temp_avg.b= b_sum/a.size();
    temp_avg.c= c_sum/a.size();
    temp_avg.d= (difference_sum/(a.size()-1));

    float d_var_sum=0.;
    float var_d=0.;
    priorexist=false;

    for(auto& elem : a)
    {
        if (priorexist ==false){
            prior = elem.key_stair.d;
            priorexist = true;
        }
        else if (priorexist == true){
            difference=(prior-elem.key_stair.d);
            cout<<"difference : "<<difference<<endl;
            d_var_sum += pow((difference-temp_avg.d),2.0);
            cout<<"d-m square sum : "<<d_var_sum<<endl;
            prior= elem.key_stair.d;
        }
    }
    var_d= d_var_sum/(a.size()-1);
    temp_avg.var_d= var_d;

    return temp_avg;
}