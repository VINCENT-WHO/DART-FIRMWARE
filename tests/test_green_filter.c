/* 只测合成观测和数学/日志接口，不使用旧曝光录像或板端运行时长。 */
#include "../h750/green_filter.c"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

static void check_covariance(const green_filter *f)
{
    double L[GF_DIM][GF_DIM]={0};
    for(unsigned i=0;i<GF_DIM;i++)for(unsigned j=0;j<=i;j++) {
        assert(isfinite(f->covariance[i][j]));
        assert(fabsf(f->covariance[i][j]-f->covariance[j][i])<1e-4f);
        double d=f->covariance[i][j];for(unsigned k=0;k<j;k++)d-=L[i][k]*L[j][k];
        if(i==j){assert(d>0);L[i][j]=sqrt(d);}else L[i][j]=d/L[j][j];
    }
}
static void test_jacobian(void)
{
    const float sample[][GF_DIM]={{120,80,10,-7,1.2f,.3f},{10,220,-40,18,.6f,-.1f},{200,20,0,0,2,1}};
    for(unsigned n=0;n<3;n++)for(unsigned t=1;t<=3;t++) {
        float F[GF_DIM][GF_DIM],a[GF_DIM];float dt=.04f*t;
        transition(sample[n],dt,a,F);
        for(unsigned j=0;j<GF_DIM;j++) {
            float plus[GF_DIM],minus[GF_DIM],p[GF_DIM],m[GF_DIM],unused[GF_DIM][GF_DIM];
            memcpy(plus,sample[n],sizeof(plus));memcpy(minus,plus,sizeof(minus));
            float epsilon=j<2 ? .1f : .001f;plus[j]+=epsilon;minus[j]-=epsilon;
            transition(plus,dt,p,unused);transition(minus,dt,m,unused);
            for(unsigned i=0;i<GF_DIM;i++)assert(fabsf((p[i]-m[i])/(2*epsilon)-F[i][j])<.015f);
        }
    }
}
static void test_lifecycle(void)
{
    green_filter f;green_filter_result r;green_filter_reset(&f);
    green_filter_step(&f,UINT32_MAX-10000,0,0,0,0,&r);assert(!r.valid && r.status==GF_WAIT);
    green_filter_step(&f,UINT32_MAX-10000,1,100,80,100,&r);
    assert(r.status==GF_INIT && r.valid && r.state[GF_S]==1 && r.area_scale==100);
    green_filter_step(&f,9999,1,100,80,100,&r);
    assert(r.dt_us==20000 && r.status==GF_UPDATE && r.valid);
    green_filter saved=f;
    green_filter_step(&f,9999,1,100,80,100,&r);
    assert(r.status==GF_TIME_INVALID && !memcmp(&saved,&f,sizeof(f)));
    green_filter_step(&f,9998,1,100,80,100,&r);assert(r.status==GF_TIME_INVALID);
    green_filter_step(&f,29999,1,310,220,6000,&r);
    assert(r.status==GF_REJECT && r.valid && r.nis>GREEN_FILTER_NIS_LIMIT);
    assert(!memcmp(r.predicted,r.state,sizeof(r.state)));check_covariance(&f);
    green_filter_step(&f,49999,1,NAN,80,100,&r);assert(r.status==GF_PREDICT && !r.measurement_valid);
    green_filter_step(&f,69999,1,100,80,-1,&r);assert(r.status==GF_PREDICT && !r.measurement_valid);
    for(uint32_t time=89999;time<529999;time+=20000)green_filter_step(&f,time,0,0,0,0,&r);
    green_filter_step(&f,529999,0,0,0,0,&r);assert(r.status==GF_LOST && !r.valid);
    green_filter_step(&f,549999,1,120,90,144,&r);assert(r.status==GF_INIT && r.area_scale==144);
    green_filter_step(&f,900000,1,121,90,144,&r);assert(r.status==GF_REINIT && r.valid);
    f.state[GF_S]=0;
    green_filter_step(&f,920000,1,122,90,144,&r);assert(r.status==GF_NUMERIC_RESET && r.valid);
    f.covariance[0][0]=NAN;
    green_filter_step(&f,940000,0,0,0,0,&r);assert(r.status==GF_NUMERIC_RESET && !r.valid);
    green_filter_reset(&f);assert(!f.active && !f.area_scale);
}
static unsigned seed=12345;
static float noise(void){seed=1664525u*seed+1013904223u;return ((seed>>8)%10001)/5000.0f-1;}
static void test_trajectory(void)
{
    green_filter f;green_filter_result r;green_filter_reset(&f);
    float truth[GF_DIM]={70,100,12,-3,1,.04f};uint32_t time=1000;
    green_filter_step(&f,time,1,truth[0],truth[1],200,&r);
    double raw_error=0,filtered_error=0;
    for(unsigned n=1;n<=500;n++) {
        uint32_t du=15000+(n%5)*4000;float dt=du*1e-6f,F[GF_DIM][GF_DIM],next[GF_DIM];
        transition(truth,dt,next,F);memcpy(truth,next,sizeof(truth));time+=du;
        float x=truth[0]+2*noise(),y=truth[1]+2*noise(),area=200*truth[4]*truth[4]+5*noise();
        int observed=n%19!=0;
        green_filter_step(&f,time,observed,x,y,area,&r);assert(r.valid);
        assert(r.status==(observed?GF_UPDATE:GF_PREDICT));check_covariance(&f);
        if(n>20 && observed){raw_error+=(x-truth[0])*(x-truth[0]);filtered_error+=(r.state[0]-truth[0])*(r.state[0]-truth[0]);}
    }
    assert(filtered_error<raw_error);
    assert(fabsf(r.state[4]-truth[4])<.05f && fabsf(r.state[2]-truth[2])<4);
    printf("PASS: synthetic variable-dt trajectory raw SSE=%.3f filtered SSE=%.3f\n",raw_error,filtered_error);
}
static unsigned commas(const char *p){unsigned n=0;for(;*p;p++)n+=*p==',';return n;}
static void test_csv(void)
{
    green_filter f;green_filter_row row={0};char b[GREEN_FILTER_CSV_LINE_CAPACITY];
    green_filter_reset(&f);row.avi_index=UINT32_MAX;row.time_us=17;row.elapsed_us=23;
    green_filter_step(&f,100,0,0,0,0,&row.estimate);
    int n=green_filter_csv_line(b,sizeof(b),1,&row);
    assert(n>0 && strstr(b,"1,,17,0,0,0,0,23,")==b);
    assert(commas(b)==30 && commas(green_filter_csv_header)==30);
    assert(!strstr(b,"nan") && !strstr(b,"inf"));
    row.measured[0]=100;row.measured[1]=80;row.measured[2]=100;
    green_filter_step(&f,200,1,100,80,100,&row.estimate);row.avi_index=0;
    n=green_filter_csv_line(b,sizeof(b),2,&row);
    assert(n>0 && strstr(b,"100.000000") && commas(b)==30 && b[n-2]=='\r' && b[n-1]=='\n');
    row.estimate.state[GF_VX]=-.123456f;row.estimate.variance[GF_X]=1e12f;
    assert(green_filter_csv_line(b,sizeof(b),3,&row)>0 && strstr(b,"-0.123456"));
    assert(green_filter_csv_line(b,8,3,&row)==-1 && b[7]==0);
    assert(green_filter_csv_line(b,0,3,&row)==-1);
}
static void test_log(void)
{
    static struct { uint32_t before;green_filter_log log;uint32_t after; } guard;
    guard.before=0x12345678;guard.after=0xabcdef01;
    green_filter_log *log=&guard.log;green_filter_row row={0};
    green_filter_log_reset(log);row.elapsed_us=UINT32_MAX;row.estimate.status=GF_PREDICT;
    row.avi_index=UINT32_MAX;row.estimate.valid=1;row.estimate.state[GF_X]=12.5f;
    for(uint32_t i=0;i<GREEN_FILTER_LOG_CAPACITY+13;i++) {
        row.time_us=i;green_filter_log_append(log,&row);
    }
    assert(log->stored==GREEN_FILTER_LOG_CAPACITY && log->dropped==13);
    assert(log->attempts==GREEN_FILTER_LOG_CAPACITY+13 && log->status_count[GF_PREDICT]==log->attempts);
    assert(log->sum_us==(uint64_t)log->attempts*UINT32_MAX && log->max_us==UINT32_MAX);
    assert(log->rows[599].time_us==599 && log->rows[599].avi_index==UINT32_MAX);
    assert(guard.before==0x12345678 && guard.after==0xabcdef01);
    green_filter_log_reset(log);
    assert(!log->stored && !log->attempts && !log->sum_us && !log->max_us && !log->dropped);
    row.time_us=42;row.elapsed_us=7;row.estimate.status=GF_INIT;green_filter_log_append(log,&row);
    assert(log->stored==1 && log->rows[0].time_us==42 && log->status_count[GF_INIT]==1);
}
int main(void)
{
    test_jacobian();test_lifecycle();test_trajectory();test_csv();test_log();
    puts("PASS: EKF Jacobian/PSD/missing/gating/time wrap/reinit/numeric recovery/CSV");return 0;
}
