#include "green_filter.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* 初始调参值，非实拍标定：按秒缩放的连续对角过程噪声强度。 */
static const float noise_rate[GF_DIM]={1.0f,1.0f,400.0f,400.0f,0.0025f,0.25f};

void green_filter_reset(green_filter *f) { memset(f,0,sizeof(*f)); }

static int state_ok(const float *x)
{
    for(unsigned i=0;i<GF_DIM;++i) if(!isfinite(x[i]) || fabsf(x[i])>1e6f)return 0;
    return x[GF_S]>=GREEN_FILTER_S_MIN && x[GF_S]<=GREEN_FILTER_S_MAX;
}
static int covariance_ok(float p[GF_DIM][GF_DIM])
{
    for(unsigned i=0;i<GF_DIM;++i) {
        for(unsigned j=0;j<GF_DIM;++j) if(!isfinite(p[i][j]) || fabsf(p[i][j])>1e12f)return 0;
        if(p[i][i]<0)return 0;
        if(p[i][i]<1e-9f)p[i][i]=1e-9f;
        for(unsigned j=0;j<i;++j) p[i][j]=p[j][i]=0.5f*(p[i][j]+p[j][i]);
    }
    return 1;
}

/* F在上一时刻状态求导，与图示dt=1模型一致，推广至秒单位真实dt。 */
static void transition(const float x[GF_DIM],float dt,float next[GF_DIM],float F[GF_DIM][GF_DIM])
{
    memset(F,0,sizeof(float)*GF_DIM*GF_DIM);
    for(unsigned i=0;i<GF_DIM;++i)F[i][i]=1.0f;
    memcpy(next,x,sizeof(float)*GF_DIM);
    float scale=1.0f+dt*x[GF_DS]/x[GF_S];
    next[GF_X]+=dt*x[GF_VX];next[GF_Y]+=dt*x[GF_VY];
    next[GF_VX]*=scale;next[GF_VY]*=scale;
    next[GF_S]+=dt*x[GF_DS];
    F[GF_X][GF_VX]=dt;F[GF_Y][GF_VY]=dt;F[GF_S][GF_DS]=dt;
    F[GF_VX][GF_VX]=F[GF_VY][GF_VY]=scale;
    F[GF_VX][GF_S]=-dt*x[GF_VX]*x[GF_DS]/(x[GF_S]*x[GF_S]);
    F[GF_VY][GF_S]=-dt*x[GF_VY]*x[GF_DS]/(x[GF_S]*x[GF_S]);
    F[GF_VX][GF_DS]=dt*x[GF_VX]/x[GF_S];
    F[GF_VY][GF_DS]=dt*x[GF_VY]/x[GF_S];
}
static int predict(green_filter *f,float dt)
{
    float F[GF_DIM][GF_DIM],tmp[GF_DIM][GF_DIM]={0},P[GF_DIM][GF_DIM]={0},next[GF_DIM];
    if(!state_ok(f->state) || !covariance_ok(f->covariance))return 0;
    transition(f->state,dt,next,F);
    if(!state_ok(next))return 0;
    for(unsigned i=0;i<GF_DIM;i++)for(unsigned j=0;j<GF_DIM;j++)for(unsigned k=0;k<GF_DIM;k++)
        tmp[i][j]+=F[i][k]*f->covariance[k][j];
    for(unsigned i=0;i<GF_DIM;i++)for(unsigned j=0;j<GF_DIM;j++) {
        for(unsigned k=0;k<GF_DIM;k++)P[i][j]+=tmp[i][k]*F[j][k];
        if(i==j)P[i][j]+=noise_rate[i]*dt;
    }
    if(!covariance_ok(P))return 0;
    memcpy(f->state,next,sizeof(next));memcpy(f->covariance,P,sizeof(P));return 1;
}

static int cholesky3(const float S[3][3],float L[3][3])
{
    memset(L,0,sizeof(float)*9);
    for(unsigned i=0;i<3;i++)for(unsigned j=0;j<=i;j++) {
        float v=S[i][j];for(unsigned k=0;k<j;k++)v-=L[i][k]*L[j][k];
        if(!isfinite(v))return 0;
        if(i==j){if(v<=1e-9f)return 0;L[i][j]=sqrtf(v);}
        else L[i][j]=v/L[j][j];
    }
    return 1;
}
static void solve3(const float L[3][3],const float b[3],float x[3])
{
    float y[3];
    for(int i=0;i<3;i++){float v=b[i];for(int j=0;j<i;j++)v-=L[i][j]*y[j];y[i]=v/L[i][i];}
    for(int i=2;i>=0;i--){float v=y[i];for(int j=i+1;j<3;j++)v-=L[j][i]*x[j];x[i]=v/L[i][i];}
}

/* H仅在x、y、s列非零，避免不必要的6x3零元素运算。
 * 不显式求逆，Cholesky求解S；Joseph形式更新协方差。 */
static int correct(green_filter *f,const float z[3],float *nis)
{
    const unsigned index[3]={GF_X,GF_Y,GF_S};
    float s=f->state[GF_S],K=f->area_scale;
    float h[3]={1,1,2.0f*K*s};
    float R[3]={GREEN_FILTER_SIGMA_X*GREEN_FILTER_SIGMA_X,
        GREEN_FILTER_SIGMA_Y*GREEN_FILTER_SIGMA_Y,
        GREEN_FILTER_SIGMA_AREA*GREEN_FILTER_SIGMA_AREA/(s*s)};
    float residual[3]={z[0]-f->state[GF_X],z[1]-f->state[GF_Y],z[2]-K*s*s};
    float S[3][3],L[3][3],PH[GF_DIM][3],gain[GF_DIM][3];
    for(unsigned i=0;i<GF_DIM;i++)for(unsigned j=0;j<3;j++)PH[i][j]=f->covariance[i][index[j]]*h[j];
    for(unsigned i=0;i<3;i++)for(unsigned j=0;j<3;j++)S[i][j]=h[i]*PH[index[i]][j]+(i==j?R[i]:0);
    if(!cholesky3(S,L))return -1;
    float weighted[3];solve3(L,residual,weighted);
    *nis=0;for(unsigned i=0;i<3;i++)*nis+=residual[i]*weighted[i];
    if(!isfinite(*nis) || *nis<0)return -1;
    if(*nis>GREEN_FILTER_NIS_LIMIT)return 0;
    for(unsigned i=0;i<GF_DIM;i++)solve3(L,PH[i],gain[i]);
    float next[GF_DIM],M[GF_DIM][GF_DIM]={0},tmp[GF_DIM][GF_DIM]={0},P[GF_DIM][GF_DIM]={0};
    for(unsigned i=0;i<GF_DIM;i++) {
        next[i]=f->state[i];M[i][i]=1;
        for(unsigned j=0;j<3;j++){next[i]+=gain[i][j]*residual[j];M[i][index[j]]-=gain[i][j]*h[j];}
    }
    if(!state_ok(next))return -1;
    for(unsigned i=0;i<GF_DIM;i++)for(unsigned j=0;j<GF_DIM;j++)for(unsigned k=0;k<GF_DIM;k++)
        tmp[i][j]+=M[i][k]*f->covariance[k][j];
    for(unsigned i=0;i<GF_DIM;i++)for(unsigned j=0;j<GF_DIM;j++) {
        for(unsigned k=0;k<GF_DIM;k++)P[i][j]+=tmp[i][k]*M[j][k];
        for(unsigned k=0;k<3;k++)P[i][j]+=gain[i][k]*R[k]*gain[j][k];
    }
    if(!covariance_ok(P))return -1;
    memcpy(f->state,next,sizeof(next));memcpy(f->covariance,P,sizeof(P));return 1;
}
static void initialize(green_filter *f,uint32_t time,const float z[3])
{
    green_filter_reset(f);
    f->area_scale=GREEN_FILTER_CALIBRATED_K>0 ? GREEN_FILTER_CALIBRATED_K : z[2];
    f->state[GF_X]=z[0];f->state[GF_Y]=z[1];f->state[GF_S]=sqrtf(z[2]/f->area_scale);
    const float diag[GF_DIM]={4,4,2500,2500,0.01f,1};
    for(unsigned i=0;i<GF_DIM;i++)f->covariance[i][i]=diag[i];
    f->active=state_ok(f->state);f->last_us=f->last_update_us=time;
}
static void snapshot(const green_filter *f,green_filter_result *r)
{
    r->valid=f->active;r->area_scale=f->area_scale;
    if(!f->active)return;
    memcpy(r->state,f->state,sizeof(r->state));
    for(unsigned i=0;i<GF_DIM;i++)r->variance[i]=f->covariance[i][i];
}
void green_filter_step(green_filter *f,uint32_t time,int observed,float x,float y,float area,green_filter_result *r)
{
    memset(r,0,sizeof(*r));r->nis=-1;
    float z[3]={x,y,area};
    int measured=observed && isfinite(x) && isfinite(y) && isfinite(area) &&
        x>=0 && x<320 && y>=0 && y<240 && area>0 && area<=76800;
    r->measurement_valid=(uint32_t)measured;
    if(!f->active) {
        if(measured){initialize(f,time,z);r->status=GF_INIT;memcpy(r->predicted,f->state,sizeof(r->predicted));}
        snapshot(f,r);return;
    }
    int32_t delta=(int32_t)(time-f->last_us);
    if(delta<=0) {
        r->status=GF_TIME_INVALID;memcpy(r->predicted,f->state,sizeof(r->predicted));snapshot(f,r);return;
    }
    r->dt_us=(uint32_t)delta;
    if((uint32_t)delta>GREEN_FILTER_MAX_DT_US || (uint32_t)(time-f->last_update_us)>GREEN_FILTER_LOST_US) {
        green_filter_reset(f);r->status=GF_LOST;
        if(measured){initialize(f,time,z);r->status=GF_REINIT;memcpy(r->predicted,f->state,sizeof(r->predicted));}
        snapshot(f,r);return;
    }
    if(!predict(f,(float)delta*1e-6f))goto numeric;
    f->last_us=time;memcpy(r->predicted,f->state,sizeof(r->predicted));
    r->status=GF_PREDICT;
    if(measured) {
        int result=correct(f,z,&r->nis);
        if(result<0)goto numeric;
        r->status=result ? GF_UPDATE : GF_REJECT;
        if(result)f->last_update_us=time;
    }
    snapshot(f,r);return;
numeric:
    green_filter_reset(f);r->status=GF_NUMERIC_RESET;r->nis=-1;
    memset(r->predicted,0,sizeof(r->predicted));
    if(measured){initialize(f,time,z);memcpy(r->predicted,f->state,sizeof(r->predicted));}
    snapshot(f,r);
}

const char green_filter_csv_header[]="\xEF\xBB\xBF"
    "attempt,avi_frame,capture_us,dt_us,status,measurement_valid,estimate_valid,filter_us,"
    "measured_x_px,measured_y_px,measured_area_px,"
    "pred_x_px,pred_y_px,pred_vx_px_s,pred_vy_px_s,pred_s,pred_ds_s,"
    "x_px,y_px,vx_px_s,vy_px_s,s,ds_s,K_area_scale,NIS,"
    "var_x,var_y,var_vx,var_vy,var_s,var_ds\r\n";

void green_filter_log_reset(green_filter_log *log)
{
    /* rows为NOLOAD内存；只访问stored以内的已完整写入行。 */
    memset(log,0,offsetof(green_filter_log,rows));
}
void green_filter_log_append(green_filter_log *log,const green_filter_row *row)
{
    log->attempts++;log->sum_us+=row->elapsed_us;
    if(row->elapsed_us>log->max_us)log->max_us=row->elapsed_us;
    if(row->estimate.status<GF_STATUS_COUNT)log->status_count[row->estimate.status]++;
    if(log->stored>=GREEN_FILTER_LOG_CAPACITY){log->dropped++;return;}
    log->rows[log->stored++]=*row;
}

/* %g由newlib-nano默认构建不支持；固定小数整数格式化，不引入printf浮点。 */
static int append(char *b,size_t cap,size_t *used,const char *fmt,...)
{
    if(*used>=cap)return -1;
    va_list args;va_start(args,fmt);int n=vsnprintf(b+*used,cap-*used,fmt,args);va_end(args);
    if(n<0 || (size_t)n>=cap-*used)return -1;
    *used+=(size_t)n;return 0;
}
static int value(char *b,size_t cap,size_t *used,float v,int present)
{
    if(!present || !isfinite(v))return append(b,cap,used,",");
    /* 数值保护上限1e12，乘1e6后仍在uint64范围内。 */
    double scaled=(double)fabsf(v)*1000000.0+0.5;
    if(scaled>1.8e19)return -1;
    uint64_t u=(uint64_t)scaled;
    char integer[21];unsigned length=0;uint64_t whole=u/1000000u;
    do{integer[length++]=(char)('0'+whole%10);whole/=10;}while(whole);
    for(unsigned i=0;i<length/2;i++){char c=integer[i];integer[i]=integer[length-1-i];integer[length-1-i]=c;}
    integer[length]=0;
    return append(b,cap,used,",%s%s.%06lu",v<0 && u ? "-" : "",integer,(unsigned long)(u%1000000u));
}
int green_filter_csv_line(char *b,size_t cap,uint32_t attempt,const green_filter_row *row)
{
    const green_filter_result *r=&row->estimate;size_t used=0;
    if(append(b,cap,&used,"%lu,",(unsigned long)attempt))return -1;
    if(row->avi_index!=UINT32_MAX && append(b,cap,&used,"%lu",(unsigned long)row->avi_index))return -1;
    if(append(b,cap,&used,",%lu,%lu,%lu,%lu,%lu,%lu",(unsigned long)row->time_us,
        (unsigned long)r->dt_us,(unsigned long)r->status,(unsigned long)r->measurement_valid,
        (unsigned long)r->valid,(unsigned long)row->elapsed_us))return -1;
    for(unsigned i=0;i<3;i++)if(value(b,cap,&used,row->measured[i],r->measurement_valid))return -1;
    for(unsigned i=0;i<GF_DIM;i++)if(value(b,cap,&used,r->predicted[i],r->valid))return -1;
    for(unsigned i=0;i<GF_DIM;i++)if(value(b,cap,&used,r->state[i],r->valid))return -1;
    if(value(b,cap,&used,r->area_scale,r->valid) || value(b,cap,&used,r->nis,r->nis>=0))return -1;
    for(unsigned i=0;i<GF_DIM;i++)if(value(b,cap,&used,r->variance[i],r->valid))return -1;
    if(append(b,cap,&used,"\r\n"))return -1;
    return (int)used;
}
