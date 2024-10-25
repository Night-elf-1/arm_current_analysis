#ifndef _LTSMC_LIB_H
#define _LTSMC_LIB_H

#define __SMC_EXPORTS
#define __stdcall


#define SMC_API //__declspec(dllimport)
#define MAX_PATH 255


#ifndef  LTUINT64
typedef  unsigned long long   LTUINT64;
#endif

#ifndef  LTINT64
typedef  long long   LTINT64;
#endif 

#ifndef BYTE
typedef unsigned char  BYTE;
#endif

#ifndef WORD
typedef unsigned short  WORD;
#endif

#ifndef  uint8
typedef unsigned char  uint8;                   
#endif

#ifndef char
typedef signed   char  int8;                    
#endif

#ifndef uint16
typedef unsigned short uint16;                  
#endif

#ifndef int16
typedef signed short int16;                   
#endif

#ifndef uint32
typedef unsigned int   uint32;               
#endif

#ifndef int32
typedef int   int32;                  
#endif

#ifndef fp32
typedef float          fp32;                    
#endif

#ifndef fp64
typedef double         fp64;                    
#endif

#ifndef uint
typedef unsigned int   uint;                  
#endif

#ifndef uint64
typedef unsigned long long uint64;
#endif
    
#ifndef int64
typedef signed long long int64;
#endif

#ifndef DWORD
typedef unsigned long  DWORD;
#endif


#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef true
#define true  1
#endif

#ifndef false
#define false 0
#endif

#ifndef NULL
#define NULL 0                 
#endif



#ifndef NULL
#ifdef  __cplusplus
#define NULL    0
#else
#define NULL    ((void *)0)
#endif
#endif



#ifdef __cplusplus
extern "C" {
#endif

#define MaxCard 255
#define MaxCrd 6
#define AXES 32
#define TracAxes 16
enum
{
	virtual_type	= 0,
	ect_type		= 1,
	rtex_type		= 2,
	pulse_type		= 3,
	can_type		= 4,
};
typedef struct
{
    volatile uint8 m_ConnectType;
    volatile uint32 m_CardType;
	volatile uint8 m_CardType2;
    volatile uint32 m_CardNetType;
	volatile int32 m_Dll_CAN_Version;
	volatile uint8 m_IfProfilePackage;
	volatile uint32 m_HardType;
    volatile uint16 TotalAxisNum;
	volatile uint16 TotalInNum;
	volatile uint16 TotalOutNum;
	volatile uint16 TotalInPortNum;
	volatile uint16 TotalOutPortNum;
    volatile uint16 TotalLinerNum;
    volatile uint32 m_FirewaveVersion;
	
    volatile uint16 TotalPulseAxisNum;
	volatile uint16 TotalAxisNumNet;
	volatile uint16 TotalInNumNet;
	volatile uint16 TotalOutNumNet;
}struct_Card;

extern struct_Card DmcDll_m_CardHandle[MaxCard];

//	
SMC_API short __stdcall smc_set_timeout(WORD ConnectNo, uint32 timems);
SMC_API short __stdcall smc_get_connect_status(WORD ConnectNo);
SMC_API short __stdcall smc_set_connect_timeout(DWORD timems);
SMC_API short __stdcall smc_set_connect_debug_time(WORD ConnectNo,DWORD time_s);
SMC_API short __stdcall smc_board_init(WORD ConnectNo,WORD type, char *pconnectstring,DWORD baud);
SMC_API short __stdcall smc_board_init_ex(WORD ConnectNo,WORD type, char *pconnectstring,DWORD dwBaudRate, DWORD dwByteSize, DWORD dwParity, DWORD dwStopBits);
SMC_API short __stdcall smc_board_close(WORD ConnectNo);
SMC_API short __stdcall smc_soft_reset(WORD ConnectNo);
SMC_API short __stdcall smc_board_reset(WORD ConnectNo);
SMC_API short __stdcall smc_get_CardInfList(WORD* CardNum,DWORD* CardTypeList,WORD* CardIdList);
SMC_API short __stdcall smc_get_card_version(WORD ConnectNo,DWORD *CardVersion);
SMC_API short __stdcall smc_get_release_version(WORD ConnectNo,char *ReleaseVersion);
SMC_API short __stdcall smc_get_card_soft_version(WORD ConnectNo,DWORD *FirmID,DWORD *SubFirmID);
SMC_API short __stdcall smc_get_card_lib_version(DWORD *LibVer);
SMC_API short __stdcall smc_get_total_axes(WORD ConnectNo,DWORD *TotalAxis);
SMC_API short __stdcall smc_get_total_ionum(WORD ConnectNo,WORD *TotalIn,WORD *TotalOut);
SMC_API short __stdcall smc_get_total_adcnum(WORD ConnectNo,WORD *TotalIn,WORD *TotalOut);
SMC_API short __stdcall smc_get_total_liners(WORD ConnectNo,DWORD *TotalLiner);
SMC_API short __stdcall smc_set_debug_mode(WORD mode,const char* FileName);
SMC_API short __stdcall smc_get_debug_mode(WORD* mode,char* FileName);

SMC_API short __stdcall smc_rtc_get_time(WORD ConnectNo,int * year, int * month, int * day, int * hour, int * min, int * sec);
SMC_API short __stdcall smc_rtc_set_time(WORD ConnectNo,int  year, int  month, int  day, int  hour, int  min, int  sec);
SMC_API short __stdcall smc_format_flash( WORD ConnectNo);
SMC_API short __stdcall smc_set_ipaddr( WORD ConnectNo,const char* IpAddr);
SMC_API short __stdcall smc_get_ipaddr( WORD ConnectNo,char* IpAddr);
SMC_API short __stdcall smc_set_com(WORD ConnectNo,WORD com, DWORD dwBaudRate, WORD wByteSize, WORD wParity, WORD wStopBits);
SMC_API short __stdcall smc_get_com(WORD ConnectNo,WORD com, DWORD* dwBaudRate, WORD* wByteSize, WORD* wParity, WORD*dwStopBits);

SMC_API short __stdcall smc_write_sn(WORD ConnectNo, uint64 sn);
SMC_API short __stdcall smc_read_sn(WORD ConnectNo, uint64* sn);
SMC_API short __stdcall smc_write_sn_numstring(WORD ConnectNo, const char* sn);
SMC_API short __stdcall smc_read_sn_numstring(WORD ConnectNo, char* sn);


SMC_API short __stdcall smc_write_password(WORD ConnectNo, const char * str_pass);
SMC_API short __stdcall smc_check_password(WORD ConnectNo, const char * str_pass);

SMC_API short __stdcall smc_enter_password(WORD ConnectNo, const char * str_pass);
SMC_API short __stdcall smc_modify_password(WORD ConnectNo, const char* spassold, const char* spass);
		
SMC_API short __stdcall smc_set_pulse_outmode(WORD ConnectNo,WORD axis,WORD outmode);
SMC_API short __stdcall smc_get_pulse_outmode(WORD ConnectNo,WORD axis,WORD* outmode);

SMC_API short __stdcall smc_set_equiv(WORD ConnectNo,WORD axis, double equiv);
SMC_API short __stdcall smc_get_equiv(WORD ConnectNo,WORD axis, double *equiv);

SMC_API short __stdcall smc_set_backlash_unit(WORD ConnectNo,WORD axis,double backlash);
SMC_API short __stdcall smc_get_backlash_unit(WORD ConnectNo,WORD axis,double *backlash);

SMC_API short __stdcall smc_set_profile_unit(WORD ConnectNo,WORD axis,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
SMC_API short __stdcall smc_get_profile_unit(WORD ConnectNo,WORD axis,double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);
SMC_API short __stdcall smc_set_profile_unit_acc(WORD ConnectNo,WORD axis,double Min_Vel,double Max_Vel,double acc,double dec,double Stop_Vel);
SMC_API short __stdcall smc_get_profile_unit_acc(WORD ConnectNo,WORD axis,double* Min_Vel,double* Max_Vel,double* acc,double* dec,double* Stop_Vel);
SMC_API short __stdcall smc_set_s_profile(WORD ConnectNo,WORD axis,WORD s_mode,double s_para);
SMC_API short __stdcall smc_get_s_profile(WORD ConnectNo,WORD axis,WORD s_mode,double *s_para);
SMC_API short __stdcall smc_set_plan_mode(WORD ConnectNo,WORD axis,WORD mode);
SMC_API short __stdcall smc_get_plan_mode(WORD ConnectNo,WORD axis,WORD *mode);

SMC_API short __stdcall smc_pmove_unit(WORD ConnectNo,WORD axis,double Dist,WORD posi_mode);
SMC_API short __stdcall smc_vmove(WORD ConnectNo,WORD axis,WORD dir);
SMC_API short __stdcall smc_change_speed_unit(WORD ConnectNo,WORD axis, double New_Vel,double Taccdec);
SMC_API short __stdcall smc_reset_target_position_unit(WORD ConnectNo,WORD axis, double New_Pos);
SMC_API short __stdcall smc_update_target_position_unit(WORD ConnectNo,WORD axis, double New_Pos);


SMC_API short __stdcall smc_sine_oscillate_unit(WORD ConnectNo,WORD Axis,double Amplitude,double Frequency);
SMC_API short __stdcall smc_sine_oscillate_stop(WORD ConnectNo,WORD Axis);
SMC_API short __stdcall smc_follow_vector_speed_move(WORD ConnectNo,WORD Axis,WORD FollowAxisNum,WORD *FollowAxisList,double ratio);

SMC_API short __stdcall smc_set_home_pin_logic(WORD ConnectNo,WORD axis,WORD org_logic,double filter);
SMC_API short __stdcall smc_get_home_pin_logic(WORD ConnectNo,WORD axis,WORD *org_logic,double *filter);
SMC_API short __stdcall smc_set_homemode(WORD ConnectNo,WORD axis,WORD home_dir,double vel_mode,WORD mode,WORD pos_source);
SMC_API short __stdcall smc_get_homemode(WORD ConnectNo,WORD axis,WORD *home_dir, double *vel_mode,WORD *home_mode,WORD *pos_source);
SMC_API short __stdcall smc_set_homespeed_unit(WORD ConnectNo,WORD axis,double speed);
SMC_API short __stdcall smc_get_homespeed_unit(WORD ConnectNo,WORD axis, double *speed);
SMC_API short __stdcall smc_set_home_profile_unit(WORD ConnectNo,WORD axis,double Low_Vel,double High_Vel,double Tacc,double Tdec);
SMC_API short __stdcall smc_get_home_profile_unit(WORD ConnectNo,WORD axis,double* Low_Vel,double* High_Vel,double* Tacc,double* Tdec);
SMC_API short __stdcall smc_home_move(WORD ConnectNo,WORD axis);
SMC_API short __stdcall smc_get_home_result(WORD ConnectNo,WORD axis,WORD* state);
SMC_API short __stdcall smc_set_home_position_unit(WORD ConnectNo,WORD axis,WORD enable,double position);
SMC_API short __stdcall smc_get_home_position_unit(WORD ConnectNo,WORD axis,WORD *enable,double *position);
SMC_API short __stdcall smc_set_el_home(WORD ConnectNo,WORD axis,WORD mode);
/*********************************************************************************************************
PVT
*********************************************************************************************************/
SMC_API short __stdcall smc_pvt_table_unit(WORD ConnectNo,WORD iaxis,DWORD count,double *pTime,double *pPos,double *pVel);
SMC_API short __stdcall smc_pts_table_unit(WORD ConnectNo,WORD iaxis,DWORD count,double *pTime,double *pPos,double *pPercent);
SMC_API short __stdcall smc_pvts_table_unit(WORD ConnectNo,WORD iaxis,DWORD count,double *pTime,double *pPos,double velBegin,double velEnd);
SMC_API short __stdcall smc_ptt_table_unit(WORD ConnectNo,WORD iaxis,DWORD count,double *pTime,double *pPos);
SMC_API short __stdcall smc_pvt_move(WORD ConnectNo,WORD AxisNum,WORD* AxisList);
/*********************************************************************************************************
*********************************************************************************************************/
SMC_API short __stdcall smc_cam_table_unit(WORD ConnectNo,WORD MasterAxisNo,WORD SlaveAxisNo,DWORD Count,double *pMasterPos,double *pSlavePos,WORD SrcMode);
SMC_API short __stdcall smc_cam_move(WORD ConnectNo,WORD AxisNo);
/*********************************************************************************************************
*********************************************************************************************************/
SMC_API short __stdcall smc_handwheel_set_axislist( WORD ConnectNo, WORD AxisSelIndex,WORD AxisNum,WORD* AxisList	);
SMC_API short __stdcall smc_handwheel_get_axislist( WORD ConnectNo,WORD AxisSelIndex, WORD* AxisNum, WORD* AxisList);
SMC_API short __stdcall smc_handwheel_set_ratiolist( WORD ConnectNo, WORD AxisSelIndex, WORD StartRatioIndex, WORD RatioSelNum, double* RatioList	);
SMC_API short __stdcall smc_handwheel_get_ratiolist( WORD ConnectNo,WORD AxisSelIndex, WORD StartRatioIndex, WORD RatioSelNum,double* RatioList );
SMC_API short __stdcall smc_handwheel_set_mode( WORD ConnectNo, WORD InMode, WORD IfHardEnable );
SMC_API short __stdcall smc_handwheel_get_mode ( WORD ConnectNo, WORD* InMode, WORD*  IfHardEnable );
SMC_API short __stdcall smc_handwheel_set_index( WORD ConnectNo, WORD AxisSelIndex,WORD RatioSelIndex );
SMC_API short __stdcall smc_handwheel_get_index( WORD ConnectNo, WORD* AxisSelIndex,WORD* RatioSelIndex );
SMC_API short __stdcall smc_handwheel_move( WORD ConnectNo, WORD ForceMove );
SMC_API short __stdcall smc_handwheel_stop ( WORD ConnectNo );
/*********************************************************************************************************
*********************************************************************************************************/	
SMC_API short __stdcall smc_set_homelatch_mode(WORD ConnectNo,WORD axis,WORD enable,WORD logic,WORD source);
SMC_API short __stdcall smc_get_homelatch_mode(WORD ConnectNo,WORD axis,WORD* enable,WORD* logic,WORD* source);
SMC_API long __stdcall smc_get_homelatch_flag(WORD ConnectNo,WORD axis);
SMC_API short __stdcall smc_reset_homelatch_flag(WORD ConnectNo,WORD axis);
SMC_API short __stdcall smc_get_homelatch_value_unit(WORD ConnectNo,WORD axis,double* pos_by_mm);
/*********************************************************************************************************
*********************************************************************************************************/
SMC_API short __stdcall smc_set_el_mode(WORD ConnectNo,WORD axis,WORD enable,WORD el_logic,WORD el_mode);
SMC_API short __stdcall smc_get_el_mode(WORD ConnectNo,WORD axis,WORD *enable,WORD *el_logic,WORD *el_mode);
SMC_API short __stdcall smc_set_emg_mode(WORD ConnectNo,WORD axis,WORD enable,WORD emg_logic);
SMC_API short __stdcall smc_get_emg_mode(WORD ConnectNo,WORD axis,WORD *enable,WORD *emg_logic);
SMC_API short __stdcall smc_set_softlimit_unit(WORD ConnectNo,WORD axis,WORD enable, WORD source_sel,WORD SL_action, double N_limit,double P_limit);
SMC_API short __stdcall smc_get_softlimit_unit(WORD ConnectNo,WORD axis,WORD *enable, WORD *source_sel,WORD *SL_action,double *N_limit,double *P_limit);
/*********************************************************************************************************
*********************************************************************************************************/
SMC_API short __stdcall smc_set_axis_io_map(WORD ConnectNo,WORD Axis,WORD IoType,WORD MapIoType,WORD MapIoIndex,double Filter);
SMC_API short __stdcall smc_get_axis_io_map(WORD ConnectNo,WORD Axis,WORD IoType,WORD* MapIoType,WORD* MapIoIndex,double* Filter);
/*********************************************************************************************************
*********************************************************************************************************/
SMC_API short __stdcall smc_set_counter_inmode(WORD ConnectNo,WORD axis,WORD mode);
SMC_API short __stdcall smc_get_counter_inmode(WORD ConnectNo,WORD axis,WORD *mode);

SMC_API short __stdcall smc_set_counter_reverse(WORD ConnectNo,WORD axis,WORD reverse);
SMC_API short __stdcall smc_get_counter_reverse(WORD ConnectNo,WORD axis,WORD *reverse);

SMC_API short __stdcall smc_set_encoder_unit(WORD ConnectNo,WORD axis, double pos);
SMC_API short __stdcall smc_get_encoder_unit(WORD ConnectNo,WORD axis, double * pos);
SMC_API short __stdcall smc_set_ez_mode(WORD ConnectNo,WORD axis,WORD ez_logic,WORD ez_mode,double filter);
SMC_API short __stdcall smc_get_ez_mode(WORD ConnectNo,WORD axis,WORD *ez_logic,WORD *ez_mode,double *filter);
SMC_API short __stdcall smc_set_ez_count(WORD ConnectNo,WORD axis,WORD ez_count);
SMC_API short __stdcall smc_get_ez_count(WORD ConnectNo,WORD axis,WORD *ez_count);

SMC_API short __stdcall smc_set_extra_encoder_mode(WORD ConnectNo,WORD channel,WORD inmode);
SMC_API short __stdcall smc_get_extra_encoder_mode(WORD ConnectNo,WORD channel,WORD* inmode);
SMC_API short __stdcall smc_set_extra_encoder(WORD ConnectNo,WORD channel, int pos);
SMC_API short __stdcall smc_get_extra_encoder(WORD ConnectNo,WORD channel, int * pos);

/*********************************************************************************************************
EZ
*********************************************************************************************************/	
SMC_API short __stdcall smc_set_ezlatch_mode(WORD ConnectNo,WORD axis,WORD enable,WORD logic,WORD source);
SMC_API short __stdcall smc_get_ezlatch_mode(WORD ConnectNo,WORD axis,WORD* enable,WORD* logic,WORD* source);
SMC_API long __stdcall smc_get_ezlatch_flag(WORD ConnectNo,WORD axis);
SMC_API short __stdcall smc_reset_ezlatch_flag(WORD ConnectNo,WORD axis);
SMC_API short __stdcall smc_get_ezlatch_value_unit(WORD ConnectNo,WORD axis,double* pos_by_mm);
/*********************************************************************************************************
*********************************************************************************************************/
SMC_API short __stdcall smc_check_done(WORD ConnectNo,WORD axis);
SMC_API short __stdcall smc_stop(WORD ConnectNo,WORD axis,WORD stop_mode);
SMC_API short __stdcall smc_emg_stop(WORD ConnectNo);
SMC_API short __stdcall smc_check_done_multicoor(WORD ConnectNo,WORD Crd);
SMC_API short __stdcall smc_stop_multicoor(WORD ConnectNo,WORD Crd,WORD stop_mode);
SMC_API DWORD __stdcall smc_axis_io_status(WORD ConnectNo,WORD axis);
SMC_API DWORD __stdcall smc_axis_io_enable_status(WORD ConnectNo,WORD axis);
SMC_API short __stdcall smc_get_axis_run_mode(WORD ConnectNo, WORD axis,WORD* run_mode);
SMC_API short __stdcall smc_read_current_speed_unit(WORD ConnectNo,WORD axis, double *current_speed);
SMC_API short __stdcall smc_set_position_unit(WORD ConnectNo,WORD axis, double pos);
SMC_API short __stdcall smc_get_position_unit(WORD ConnectNo,WORD axis, double * pos);
SMC_API short __stdcall smc_get_target_position_unit(WORD ConnectNo,WORD axis, double * pos);
SMC_API short __stdcall smc_set_workpos_unit(WORD ConnectNo,WORD axis, double pos);
SMC_API short __stdcall smc_get_workpos_unit(WORD ConnectNo,WORD axis, double *pos);
SMC_API short __stdcall smc_get_stop_reason(WORD ConnectNo,WORD axis,long* StopReason);
SMC_API short __stdcall smc_clear_stop_reason(WORD ConnectNo,WORD axis);
/*********************************************************************************************************
*********************************************************************************************************/	
SMC_API short __stdcall smc_read_inbit(WORD ConnectNo,WORD bitno);
SMC_API short __stdcall smc_write_outbit(WORD ConnectNo,WORD bitno,WORD on_off);
SMC_API short __stdcall smc_read_outbit(WORD ConnectNo,WORD bitno);
SMC_API DWORD __stdcall smc_read_inport(WORD ConnectNo,WORD portno);
SMC_API DWORD __stdcall smc_read_outport(WORD ConnectNo,WORD portno);

SMC_API short __stdcall smc_read_inbit_ex(WORD ConnectNo,WORD bitno,WORD *state);
SMC_API short __stdcall smc_read_outbit_ex(WORD ConnectNo,WORD bitno,WORD *state);
SMC_API short __stdcall smc_read_inport_ex(WORD ConnectNo,WORD portno,DWORD *state);
SMC_API short __stdcall smc_read_outport_ex(WORD ConnectNo,WORD portno,DWORD *state);

SMC_API short __stdcall smc_write_outport(WORD ConnectNo,WORD portno,DWORD outport_val);
SMC_API short __stdcall smc_reverse_outbit(WORD ConnectNo,WORD bitno,double reverse_time);

SMC_API short __stdcall smc_set_io_pwmoutput(WORD ConnectNo,WORD outbit,double time1, double time2, DWORD counts);

SMC_API short __stdcall smc_clear_io_pwmoutput(WORD ConnectNo,WORD outbit);

SMC_API short __stdcall smc_set_io_count_mode(WORD ConnectNo,WORD bitno,WORD mode,double filter);
SMC_API short __stdcall smc_get_io_count_mode(WORD ConnectNo,WORD bitno,WORD *mode,double* filter);
SMC_API short __stdcall smc_set_io_count_value(WORD ConnectNo,WORD bitno,DWORD CountValue);
SMC_API short __stdcall smc_get_io_count_value(WORD ConnectNo,WORD bitno,DWORD* CountValue);
SMC_API short __stdcall smc_set_io_dstp_mode(WORD ConnectNo,WORD axis,WORD enable,WORD logic); 
SMC_API short __stdcall smc_get_io_dstp_mode(WORD ConnectNo,WORD axis,WORD *enable,WORD *logic); 	
SMC_API short __stdcall smc_set_dec_stop_time(WORD ConnectNo,WORD axis,double time);
SMC_API short __stdcall smc_get_dec_stop_time(WORD ConnectNo,WORD axis,double *time); 

SMC_API short __stdcall smc_set_io_map_virtual(WORD ConnectNo,WORD bitno,WORD MapIoType,WORD MapIoIndex,double Filter);
SMC_API short __stdcall smc_get_io_map_virtual(WORD ConnectNo,WORD bitno,WORD* MapIoType,WORD* MapIoIndex,double* Filter);
SMC_API short __stdcall smc_read_inbit_virtual(WORD ConnectNo,WORD bitno); 
/*********************************************************************************************************
*********************************************************************************************************/	
SMC_API short __stdcall smc_set_alm_mode(WORD ConnectNo,WORD axis,WORD enable,WORD alm_logic,WORD alm_action);
SMC_API short __stdcall smc_get_alm_mode(WORD ConnectNo,WORD axis,WORD *enable,WORD *alm_logic,WORD *alm_action);
SMC_API short __stdcall smc_set_inp_mode(WORD ConnectNo,WORD axis,WORD enable,WORD inp_logic);
SMC_API short __stdcall smc_get_inp_mode(WORD ConnectNo,WORD axis,WORD *enable,WORD *inp_logic);

SMC_API short __stdcall smc_write_sevon_pin(WORD ConnectNo,WORD axis,WORD on_off);
SMC_API short __stdcall smc_read_sevon_pin(WORD ConnectNo,WORD axis);
SMC_API short __stdcall smc_write_erc_pin(WORD ConnectNo,WORD axis,WORD on_off);
SMC_API short __stdcall smc_read_erc_pin(WORD ConnectNo,WORD axis); 
SMC_API short __stdcall smc_write_sen_pin(WORD ConnectNo,WORD axis,WORD on_off);
SMC_API short __stdcall smc_read_sen_pin(WORD ConnectNo,WORD axis); 	
SMC_API short __stdcall smc_read_alarm_pin(WORD ConnectNo,WORD axis);
SMC_API short __stdcall smc_read_inp_pin(WORD ConnectNo,WORD axis);
SMC_API short __stdcall smc_read_org_pin(WORD ConnectNo,WORD uiaxis);
SMC_API short __stdcall smc_read_elp_pin(WORD ConnectNo,WORD uiaxis);
SMC_API short __stdcall smc_read_eln_pin(WORD ConnectNo,WORD uiaxis);
SMC_API short __stdcall smc_read_emg_pin(WORD ConnectNo,WORD uiaxis);
SMC_API short __stdcall smc_read_ez_pin(WORD ConnectNo,WORD axis);
SMC_API short __stdcall smc_read_rdy_pin(WORD ConnectNo,WORD axis);

SMC_API short __stdcall smc_read_sevon_pin_ex(WORD ConnectNo,WORD axis,WORD *state);
SMC_API short __stdcall smc_read_erc_pin_ex(WORD ConnectNo,WORD axis,WORD *state); 
SMC_API short __stdcall smc_read_alarm_pin_ex(WORD ConnectNo,WORD axis,WORD *state);
SMC_API short __stdcall smc_read_inp_pin_ex(WORD ConnectNo,WORD axis,WORD *state);
SMC_API short __stdcall smc_read_org_pin_ex(WORD ConnectNo,WORD uiaxis,WORD *state);
SMC_API short __stdcall smc_read_elp_pin_ex(WORD ConnectNo,WORD uiaxis,WORD *state);
SMC_API short __stdcall smc_read_eln_pin_ex(WORD ConnectNo,WORD uiaxis,WORD *state);
SMC_API short __stdcall smc_read_emg_pin_ex(WORD ConnectNo,WORD uiaxis,WORD *state);
SMC_API short __stdcall smc_read_ez_pin_ex(WORD ConnectNo,WORD axis,WORD *state);
SMC_API short __stdcall smc_read_rdy_pin_ex(WORD ConnectNo,WORD axis,WORD *state);

SMC_API short __stdcall smc_set_sevon_enable(WORD ConnectNo,WORD axis,WORD on_off);
SMC_API short __stdcall smc_get_sevon_enable(WORD ConnectNo,WORD axis,WORD* on_off);
/*********************************************************************************************************
*********************************************************************************************************/
SMC_API short __stdcall smc_set_vector_tacc(WORD ConnectNo,WORD Crd,double Tacc);
SMC_API short __stdcall smc_set_vector_acc(WORD ConnectNo,WORD Crd,double acc);
SMC_API short __stdcall smc_set_vector_speed_unit(WORD ConnectNo,WORD Crd,double Max_vel);
SMC_API short __stdcall smc_get_vector_tacc(WORD ConnectNo,WORD Crd,double* Tacc);
SMC_API short __stdcall smc_get_vector_acc(WORD ConnectNo,WORD Crd,double* acc);
SMC_API short __stdcall smc_get_vector_speed_unit(WORD ConnectNo,WORD Crd,double* Max_vel);

SMC_API short __stdcall smc_set_vector_profile_unit(WORD ConnectNo,WORD Crd,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
SMC_API short __stdcall smc_get_vector_profile_unit(WORD ConnectNo,WORD Crd,double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);
SMC_API short __stdcall smc_set_vector_profile_unit_acc(WORD ConnectNo,WORD Crd,double Min_Vel,double Max_Vel,double acc,double dec,double Stop_Vel);
SMC_API short __stdcall smc_get_vector_profile_unit_acc(WORD ConnectNo,WORD Crd,double* Min_Vel,double* Max_Vel,double* acc,double* dec,double* Stop_Vel);
SMC_API short __stdcall smc_set_vector_s_profile(WORD ConnectNo,WORD Crd,WORD s_mode,double s_para);
SMC_API short __stdcall smc_get_vector_s_profile(WORD ConnectNo,WORD Crd,WORD s_mode,double *s_para);
SMC_API short __stdcall smc_set_vector_dec_stop_time(WORD ConnectNo,WORD Crd,double time);
SMC_API short __stdcall smc_get_vector_dec_stop_time(WORD ConnectNo,WORD Crd,double *time);

SMC_API short __stdcall smc_line_unit(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Dist,WORD posi_mode);
SMC_API short __stdcall smc_arc_move_center_unit(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Cen_Pos,WORD Arc_Dir,long Circle,WORD posi_mode);
SMC_API short __stdcall smc_arc_move_radius_unit(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double Arc_Radius,WORD Arc_Dir,long Circle,WORD posi_mode);
SMC_API short __stdcall smc_arc_move_3points_unit(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Mid_Pos,long Circle,WORD posi_mode);


SMC_API short __stdcall smc_conti_open_list (WORD ConnectNo,WORD Crd,WORD AxisNum,WORD *AxisList);
SMC_API short __stdcall smc_conti_close_list(WORD ConnectNo,WORD Crd);
SMC_API short __stdcall smc_conti_stop_list (WORD ConnectNo,WORD Crd,WORD stop_mode);
SMC_API short __stdcall smc_conti_stop_list_by_vel (WORD ConnectNo,WORD Crd,WORD stop_mode,double stop_vel);
SMC_API short __stdcall smc_conti_pause_list(WORD ConnectNo,WORD Crd);
SMC_API short __stdcall smc_conti_start_list(WORD ConnectNo,WORD Crd);
SMC_API short __stdcall smc_conti_check_done(WORD ConnectNo,WORD Crd);
SMC_API short __stdcall smc_conti_get_run_state(WORD ConnectNo,WORD Crd);
SMC_API long __stdcall smc_conti_remain_space (WORD ConnectNo,WORD Crd);
SMC_API long __stdcall smc_conti_read_current_mark (WORD ConnectNo,WORD Crd);
SMC_API short __stdcall smc_conti_change_speed_ratio (WORD ConnectNo,WORD Crd,double percent);

SMC_API short __stdcall smc_conti_set_blend(WORD ConnectNo,WORD Crd,WORD enable);
SMC_API short __stdcall smc_conti_get_blend(WORD ConnectNo,WORD Crd,WORD* enable);

SMC_API short __stdcall smc_conti_set_override(WORD ConnectNo,WORD Crd,double Percent);

SMC_API short __stdcall smc_conti_wait_input(WORD ConnectNo,WORD Crd,WORD bitno,WORD on_off,double TimeOut,long mark);
SMC_API short __stdcall smc_conti_wait_input_action(WORD ConnectNo,WORD Crd,WORD bitno,WORD on_off,double TimeOut,WORD action,long mark);
SMC_API short __stdcall smc_conti_delay_outbit_to_start(WORD ConnectNo, WORD Crd, WORD bitno,WORD on_off,double delay_value,WORD delay_mode,double ReverseTime);
SMC_API short __stdcall smc_conti_delay_outbit_to_stop(WORD ConnectNo, WORD Crd, WORD bitno,WORD on_off,double delay_time,double ReverseTime);
SMC_API short __stdcall smc_conti_ahead_outbit_to_stop(WORD ConnectNo, WORD Crd, WORD bitno,WORD on_off,double ahead_value,WORD ahead_mode,double ReverseTime);
SMC_API short __stdcall smc_conti_accurate_outbit_unit(WORD ConnectNo, WORD Crd, WORD cmp_no,WORD on_off,WORD axis,double abs_pos,WORD pos_source,double ReverseTime);
SMC_API short __stdcall smc_conti_write_outbit(WORD ConnectNo, WORD Crd, WORD bitno,WORD on_off,double ReverseTime);
SMC_API short __stdcall smc_conti_clear_io_action(WORD ConnectNo, WORD Crd, DWORD Io_Mask);
SMC_API short __stdcall smc_conti_set_pause_output(WORD ConnectNo,WORD Crd,WORD action,long mask,long state);
SMC_API short __stdcall smc_conti_get_pause_output(WORD ConnectNo,WORD Crd,WORD* action,long* mask,long* state);

SMC_API short __stdcall smc_conti_line_unit(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* pPosList,WORD posi_mode,long mark);
SMC_API short __stdcall smc_conti_arc_move_center_unit(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Cen_Pos,WORD Arc_Dir,long Circle,WORD posi_mode,long mark);
SMC_API short __stdcall smc_conti_arc_move_radius_unit(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double Arc_Radius,WORD Arc_Dir,long Circle,WORD posi_mode,long mark);
SMC_API short __stdcall smc_conti_arc_move_3points_unit(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Mid_Pos,long Circle,WORD posi_mode,long mark);
SMC_API short __stdcall smc_conti_pmove_unit(WORD ConnectNo,WORD Crd,WORD axis,double dist,WORD posi_mode,WORD mode,long mark);
SMC_API short __stdcall smc_conti_delay(WORD ConnectNo, WORD Crd,double delay_time,long mark);
SMC_API short __stdcall smc_conti_nurbs_move_unit(WORD ConnectNo,WORD Crd, WORD *AxisList,double* pPosListX,double* pPosListY,DWORD PosNum,DWORD InsertNum,WORD posi_mode,long mark);
SMC_API short __stdcall smc_nurbs_move_unit(WORD ConnectNo,WORD Crd, WORD *AxisList,double* pPosListX,double* pPosListY,DWORD PosNum,DWORD InsertNum,WORD posi_mode,long mark);
SMC_API short __stdcall BSpline3_Calculate(double* pPos1,double* pPos2,DWORD PosNum,DWORD InsertNum,double *X_POS,double *Y_POS,DWORD *Count);

SMC_API short __stdcall smc_conti_open_list_ex (WORD ConnectNo,WORD Crd,WORD AxisNum,WORD *AxisList);
SMC_API short __stdcall smc_conti_close_list_ex(WORD ConnectNo,WORD Crd);
SMC_API short __stdcall smc_conti_line_unit_ex(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Dist,WORD posi_mode,long mark,double ExtendLength);
SMC_API short __stdcall smc_conti_line_unit_ex2(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* pPosList,WORD posi_mode,long mark);

SMC_API short __stdcall smc_conti_line_io_union(WORD CardNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* pPosList,WORD posi_mode,
	WORD bitno,WORD on_off,double io_value,WORD io_mode,WORD MapAxis,WORD pos_source,double ReverseTime,long mark);
SMC_API short __stdcall smc_conti_add_io_action(WORD ConnectNo,WORD Crd,WORD bitno,WORD on_off,double io_value,WORD io_mode,WORD MapAxis,WORD pos_source,double ReverseTime);

SMC_API short __stdcall smc_rtcp_set_kinematic_type(WORD ConnectNo,WORD Crd, WORD machine_type);
SMC_API short __stdcall smc_rtcp_get_kinematic_type(WORD ConnectNo,WORD Crd, WORD * machine_type);

SMC_API short __stdcall smc_rtcp_set_enable(WORD ConnectNo,WORD Crd, WORD enable);
SMC_API short __stdcall smc_rtcp_get_enable(WORD ConnectNo,WORD Crd, WORD * enable);

SMC_API short __stdcall smc_rtcp_set_vector_a(WORD ConnectNo,WORD Crd, double * a_offset);
SMC_API short __stdcall smc_rtcp_get_vector_a(WORD ConnectNo,WORD Crd, double * a_offset);

SMC_API short __stdcall smc_rtcp_set_vector_b(WORD ConnectNo,WORD Crd, double * b_offset);
SMC_API short __stdcall smc_rtcp_get_vector_b(WORD ConnectNo,WORD Crd, double * b_offset);

SMC_API short __stdcall smc_rtcp_set_vector_c(WORD ConnectNo,WORD Crd, double * c_offset);
SMC_API short __stdcall smc_rtcp_get_vector_c(WORD ConnectNo,WORD Crd, double * c_offset);

SMC_API short __stdcall smc_rtcp_set_rotate_joint_offset(WORD ConnectNo,WORD Crd, double A, double B, double C);
SMC_API short __stdcall smc_rtcp_get_rotate_joint_offset(WORD ConnectNo,WORD Crd, double * A, double * B, double * C);

SMC_API short __stdcall smc_rtcp_set_joints_direction(WORD ConnectNo,WORD Crd, int * dir);
SMC_API short __stdcall smc_rtcp_get_joints_direction(WORD ConnectNo,WORD Crd, int * dir);

SMC_API short __stdcall smc_rtcp_set_tool_length(WORD ConnectNo,WORD Crd, double tool);
SMC_API short __stdcall smc_rtcp_get_tool_length(WORD ConnectNo,WORD Crd, double * tool);

SMC_API short __stdcall smc_rtcp_get_actual_pos(WORD ConnectNo,WORD Crd, WORD AxisNum,double * actual_pos);
SMC_API short __stdcall smc_rtcp_get_command_pos(WORD ConnectNo,WORD Crd, WORD AxisNum,double * command_pos);


SMC_API short __stdcall smc_get_distance_to_start(WORD CardNo,WORD Crd, double* distance_x, double* distance_y,long imark); 

SMC_API short __stdcall smc_set_start_distance_flag(WORD CardNo,WORD Crd,WORD flag); 
SMC_API short __stdcall smc_get_start_distance_flag(WORD CardNo,WORD Crd,WORD *flag); 
/*********************************************************************************************************
*********************************************************************************************************/
SMC_API short __stdcall smc_set_pwm_enable(WORD ConnectNo,WORD pwm,WORD enable);
SMC_API short __stdcall smc_get_pwm_enable(WORD ConnectNo,WORD pwm,WORD* enable);
SMC_API short __stdcall smc_set_pwm_output(WORD ConnectNo, WORD PwmNo,double fDuty, double fFre);
SMC_API short __stdcall smc_get_pwm_output(WORD ConnectNo,WORD PwmNo,double* fDuty, double* fFre);
SMC_API short __stdcall smc_conti_set_pwm_output(WORD ConnectNo,WORD Crd, WORD PwmNo,double fDuty, double fFre);
//
SMC_API short __stdcall smc_set_pwm_output_num(WORD ConnectNo, WORD PwmNo,double fDuty, double fFre,DWORD num);
SMC_API short __stdcall smc_force_pwm_output_level(WORD ConnectNo, WORD PwmNo,WORD  level); 
SMC_API short __stdcall smc_get_pwm_output_state(WORD ConnectNo, WORD PwmNo,WORD *state,WORD *level,WORD  *num);
/**********PWM**********************************************************************************
*******************************************************************************************************/
SMC_API short __stdcall smc_set_pwm_follow_speed(WORD ConnectNo,WORD pwm_no,WORD mode,double MaxVel,double MaxValue,double OutValue);
SMC_API short __stdcall smc_get_pwm_follow_speed(WORD ConnectNo,WORD pwm_no,WORD* mode,double* MaxVel,double* MaxValue,double* OutValue);

SMC_API short __stdcall smc_set_pwm_onoff_duty(WORD ConnectNo, WORD PwmNo,double fOnDuty, double fOffDuty);
SMC_API short __stdcall smc_get_pwm_onoff_duty(WORD ConnectNo, WORD PwmNo,double* fOnDuty, double* fOffDuty);

SMC_API short __stdcall smc_set_pwm_follow_onoff(WORD ConnectNo, WORD PwmNo,WORD Crd,WORD on_off);
SMC_API short __stdcall smc_get_pwm_follow_onoff(WORD ConnectNo, WORD PwmNo,WORD *Crd,WORD *on_off);

SMC_API short __stdcall smc_conti_delay_pwm_to_start(WORD ConnectNo, WORD Crd, WORD pwmno,WORD on_off,double delay_value,WORD delay_mode,double ReverseTime);
SMC_API short __stdcall smc_conti_ahead_pwm_to_stop(WORD ConnectNo, WORD Crd, WORD bitno,WORD on_off,double ahead_value,WORD ahead_mode,double ReverseTime);
SMC_API short __stdcall smc_conti_write_pwm(WORD ConnectNo, WORD Crd, WORD pwmno,WORD on_off,double ReverseTime);


SMC_API short __stdcall smc_conti_set_lookahead_mode(WORD ConnectNo,WORD Crd,WORD enable,long LookaheadSegments,double PathError,double LookaheadAcc);
SMC_API short __stdcall smc_conti_get_lookahead_mode(WORD ConnectNo,WORD Crd,WORD* enable,long* LookaheadSegments,double* PathError,double* LookaheadAcc);

SMC_API short __stdcall smc_set_arc_limit(WORD ConnectNo,WORD Crd,WORD Enable,double MaxCenAcc,double MaxArcError);
SMC_API short __stdcall smc_get_arc_limit(WORD ConnectNo,WORD Crd,WORD* Enable,double* MaxCenAcc,double* MaxArcError);

/*********************************************************************************************************
*********************************************************************************************************/
SMC_API short __stdcall smc_compare_set_config(WORD ConnectNo,WORD axis,WORD enable, WORD cmp_source); 	
SMC_API short __stdcall smc_compare_get_config(WORD ConnectNo,WORD axis,WORD *enable, WORD *cmp_source);	
SMC_API short __stdcall smc_compare_clear_points(WORD ConnectNo,WORD axis); 	
SMC_API short __stdcall smc_compare_add_point_unit(WORD ConnectNo,WORD axis,double pos,WORD dir, WORD action,DWORD actpara); 	

SMC_API short __stdcall smc_compare_add_point_cycle(WORD CardNo,WORD cmp,double pos,WORD dir, DWORD bitno,DWORD cycle,WORD level);
SMC_API short __stdcall smc_compare_get_current_point_unit(WORD ConnectNo,WORD axis,double *pos); 	
SMC_API short __stdcall smc_compare_get_points_runned(WORD ConnectNo,WORD axis,long *pointNum); 	
SMC_API short __stdcall smc_compare_get_points_remained(WORD ConnectNo,WORD axis,long *pointNum); 	

SMC_API short __stdcall smc_compare_set_config_extern(WORD ConnectNo,WORD enable, WORD cmp_source); 	
SMC_API short __stdcall smc_compare_get_config_extern(WORD ConnectNo,WORD *enable, WORD *cmp_source);	
SMC_API short __stdcall smc_compare_clear_points_extern(WORD ConnectNo); 	
SMC_API short __stdcall smc_compare_add_point_extern_unit(WORD ConnectNo,WORD* axis,double* pos,WORD* dir, WORD action,DWORD actpara); 	
SMC_API short __stdcall smc_compare_add_point_cycle_2d(WORD ConnectNo,WORD* axis,double* pos,WORD* dir, DWORD bitno,DWORD cycle,WORD level); 	
SMC_API short __stdcall smc_compare_get_current_point_extern_unit(WORD ConnectNo,double *pos); 
SMC_API short __stdcall smc_compare_get_points_runned_extern(WORD ConnectNo,long *pointNum); 	
SMC_API short __stdcall smc_compare_get_points_remained_extern(WORD ConnectNo,long *pointNum); 

SMC_API short __stdcall smc_hcmp_set_mode(WORD ConnectNo,WORD hcmp, WORD cmp_mode);
SMC_API short __stdcall smc_hcmp_get_mode(WORD ConnectNo,WORD hcmp, WORD* cmp_mode);
SMC_API short __stdcall smc_hcmp_set_config(WORD ConnectNo,WORD hcmp,WORD axis, WORD cmp_source, WORD cmp_logic,long time);
SMC_API short __stdcall smc_hcmp_get_config(WORD ConnectNo,WORD hcmp,WORD* axis, WORD* cmp_source, WORD* cmp_logic,long* time);
SMC_API short __stdcall smc_hcmp_add_point_unit(WORD ConnectNo,WORD hcmp, double cmp_pos);
SMC_API short __stdcall smc_hcmp_set_liner_unit(WORD ConnectNo,WORD hcmp, double Increment,long Count);
SMC_API short __stdcall smc_hcmp_get_liner_unit(WORD ConnectNo,WORD hcmp, double* Increment,long* Count);
SMC_API short __stdcall smc_hcmp_get_current_state_unit(WORD ConnectNo,WORD hcmp,long *remained_points,double *current_point,long *runned_points); 
SMC_API short __stdcall smc_hcmp_clear_points(WORD ConnectNo,WORD hcmp);

SMC_API short __stdcall smc_read_cmp_pin(WORD ConnectNo,WORD axis);
SMC_API short __stdcall smc_write_cmp_pin(WORD ConnectNo,WORD axis, WORD on_off);


SMC_API short __stdcall smc_hcmp_fifo_set_mode(WORD ConnectNo,WORD hcmp, WORD fifo_mode);
SMC_API short __stdcall smc_hcmp_fifo_get_mode(WORD ConnectNo,WORD hcmp, WORD* fifo_mode);

SMC_API short __stdcall smc_hcmp_fifo_get_state(WORD ConnectNo,WORD hcmp,long *remained_points); 

SMC_API short __stdcall smc_hcmp_fifo_add_point_unit(WORD ConnectNo,WORD hcmp, WORD num,double *cmp_pos);

SMC_API short __stdcall smc_hcmp_fifo_clear_points(WORD ConnectNo,WORD hcmp);

SMC_API short __stdcall smc_hcmp_fifo_add_table(WORD ConnectNo,WORD hcmp, WORD num,double *cmp_pos);


SMC_API short __stdcall smc_hcmp_2d_set_enable(WORD ConnectNo,WORD hcmp, WORD cmp_enable);
SMC_API short __stdcall smc_hcmp_2d_get_enable(WORD ConnectNo,WORD hcmp, WORD *cmp_enable);


SMC_API short __stdcall smc_hcmp_2d_set_config_unit(WORD ConnectNo,WORD hcmp,WORD cmp_mode,WORD x_axis, WORD x_cmp_source, double x_cmp_error, WORD y_axis, WORD y_cmp_source, double y_cmp_error,WORD cmp_logic,int time);
SMC_API short __stdcall smc_hcmp_2d_get_config_unit(WORD ConnectNo,WORD hcmp,WORD *cmp_mode,WORD *x_axis, WORD *x_cmp_source,double *x_cmp_error,  WORD *y_axis, WORD *y_cmp_source, double *y_cmp_error, WORD *cmp_logic,int *time);

SMC_API short __stdcall smc_hcmp_2d_set_pwmoutput(WORD ConnectNo,WORD hcmp,WORD pwm_enable,double duty,double freq,WORD pwm_number);
SMC_API short __stdcall smc_hcmp_2d_get_pwmoutput(WORD ConnectNo,WORD hcmp,WORD *pwm_enable,double *duty,double *freq,WORD *pwm_number);
SMC_API short __stdcall smc_hcmp_2d_add_point_unit(WORD ConnectNo,WORD hcmp, double x_cmp_pos, double y_cmp_pos,WORD cmp_outbit);


SMC_API short __stdcall smc_hcmp_2d_get_current_state_unit(WORD ConnectNo,WORD hcmp,int *remained_points,double *x_current_point,double *y_current_point,int *runned_points,WORD *current_state,WORD *current_outbit); 
SMC_API short __stdcall smc_hcmp_2d_get_current_state(WORD ConnectNo,WORD hcmp,int *remained_points,double *x_current_point,double *y_current_point,int *runned_points,WORD *current_state,WORD *current_outbit); 


SMC_API short __stdcall smc_hcmp_2d_clear_points(WORD ConnectNo,WORD hcmp);
SMC_API short __stdcall smc_hcmp_2d_force_output(WORD ConnectNo,WORD hcmp,WORD outbit);


SMC_API short __stdcall smc_ltc_set_mode(WORD ConnectNo,WORD latch,WORD ltc_mode,WORD ltc_logic,double filter);
SMC_API short __stdcall smc_ltc_get_mode(WORD ConnectNo,WORD latch,WORD *ltc_mode,WORD *ltc_logic,double *filter);

SMC_API short __stdcall smc_ltc_set_source(WORD ConnectNo,WORD latch,WORD axis,WORD ltc_source);
SMC_API short __stdcall smc_ltc_get_source(WORD ConnectNo,WORD latch,WORD axis,WORD *ltc_source);

SMC_API short __stdcall smc_ltc_reset(WORD ConnectNo,WORD latch);

SMC_API short __stdcall smc_ltc_get_number(WORD ConnectNo,WORD latch,WORD axis,int *number);

SMC_API short __stdcall smc_ltc_get_value_unit(WORD ConnectNo,WORD latch,WORD axis,double *value);


SMC_API short __stdcall smc_softltc_set_mode(WORD ConnectNo,WORD latch,WORD ltc_enable,WORD ltc_mode,WORD ltc_inbit,WORD ltc_logic,double filter);
SMC_API short __stdcall smc_softltc_get_mode(WORD ConnectNo,WORD latch,WORD *ltc_enable,WORD *ltc_mode,WORD *ltc_inbit,WORD *ltc_logic,double *filter);

SMC_API short __stdcall smc_softltc_set_source(WORD ConnectNo,WORD latch,WORD axis,WORD ltc_source);
SMC_API short __stdcall smc_softltc_get_source(WORD ConnectNo,WORD latch,WORD axis,WORD *ltc_source);

SMC_API short __stdcall smc_softltc_reset(WORD ConnectNo,WORD latch);

SMC_API short __stdcall smc_softltc_get_number(WORD ConnectNo,WORD latch,WORD axis,int *number);

SMC_API short __stdcall smc_softltc_get_value_unit(WORD ConnectNo,WORD latch,WORD axis,double *value);

SMC_API short __stdcall smc_set_ltc_mode(WORD ConnectNo,WORD ltc,WORD ltc_logic,WORD ltc_mode,double filter);
SMC_API short __stdcall smc_get_ltc_mode(WORD ConnectNo,WORD ltc,WORD* ltc_logic,WORD* ltc_mode,double *filter);
SMC_API short __stdcall smc_set_latch_mode(WORD ConnectNo,WORD axis,WORD latch_mode,WORD latch_source,WORD triger_chunnel);
SMC_API short __stdcall smc_get_latch_mode(WORD ConnectNo,WORD axis,WORD* latch_mode,WORD* latch_source,WORD* triger_chunnel);
SMC_API short __stdcall smc_get_latch_flag(WORD ConnectNo,WORD axis);
SMC_API short __stdcall smc_reset_latch_flag(WORD ConnectNo,WORD axis);
SMC_API short __stdcall smc_get_latch_value_unit(WORD ConnectNo,WORD axis,double* pos_by_mm);

SMC_API double __stdcall smc_get_ain(WORD ConnectNo,WORD channel);
SMC_API short __stdcall smc_set_ain_action(WORD ConnectNo,WORD channel,WORD mode,double fvoltage,WORD action,double actpara);
SMC_API short __stdcall smc_get_ain_action(WORD ConnectNo,WORD channel,WORD* mode,double* fvoltage,WORD* action,double* actpara);
SMC_API short __stdcall smc_get_ain_state(WORD ConnectNo,WORD channel);
SMC_API short __stdcall smc_set_ain_state(WORD ConnectNo,WORD channel);

SMC_API short __stdcall smc_set_da_output(WORD ConnectNo, WORD channel,double Vout);
SMC_API short __stdcall smc_get_da_output(WORD ConnectNo,WORD channel,double* Vout);

SMC_API short __stdcall smc_laser_set_output(WORD ConnectNo,WORD Enable,WORD Width);
SMC_API short __stdcall smc_laser_get_output(WORD ConnectNo,WORD* Enable,WORD* Width);

SMC_API short __stdcall smc_download_file(WORD ConnectNo, const char* pfilename, const char* pfilenameinControl,WORD filetype);
SMC_API short __stdcall smc_download_memfile(WORD ConnectNo, const char* pbuffer, uint32 buffsize, const char* pfilenameinControl,WORD filetype);
SMC_API short __stdcall smc_upload_file(WORD ConnectNo, const char* pfilename, const char* pfilenameinControl, WORD filetype);
SMC_API short __stdcall smc_upload_memfile(WORD ConnectNo, char* pbuffer, uint32 buffsize, const char* pfilenameinControl, uint32* puifilesize,WORD filetype);
SMC_API short __stdcall smc_download_file_to_ram(WORD ConnectNo, const char* pfilename,WORD filetype);
SMC_API short __stdcall smc_download_memfile_to_ram(WORD ConnectNo, const char* pbuffer, uint32 buffsize,WORD filetype);
SMC_API short __stdcall smc_get_progress(WORD ConnectNo,float* process);

SMC_API short __stdcall smc_download_parafile(WORD ConnectNo,const char *FileName);
SMC_API short __stdcall smc_upload_parafile(WORD ConnectNo,const char *FileName);

SMC_API short __stdcall smc_udisk_get_state(WORD ConnectNo,WORD *state);
SMC_API short __stdcall smc_udisk_check_file(WORD ConnectNo,const char* filename,int* filesize,WORD filetype);
SMC_API short __stdcall smc_udisk_get_first_file(WORD ConnectNo,char* filename,int* filesize,int* fileid,WORD filetype);
SMC_API short __stdcall smc_udisk_get_next_file(WORD ConnectNo,char* filename,int* filesize,int* fileid,WORD filetype);
SMC_API short __stdcall smc_udisk_copy_file(WORD ConnectNo,const char* SrcFileName,const char* DstFileName,WORD filetype,WORD mode);


SMC_API short __stdcall smc_set_modbus_0x(WORD ConnectNo, WORD start, WORD inum, const char* pdata);
SMC_API short __stdcall smc_get_modbus_0x(WORD ConnectNo, WORD start, WORD inum, char* pdata);
SMC_API short __stdcall smc_set_modbus_4x(WORD ConnectNo, WORD start, WORD inum, const WORD* pdata);
SMC_API short __stdcall smc_get_modbus_4x(WORD ConnectNo, WORD start, WORD inum, WORD* pdata);

SMC_API short __stdcall smc_set_modbus_4x_float(WORD ConnectNo, WORD start, WORD inum, const float* pdata);
SMC_API short __stdcall smc_get_modbus_4x_float(WORD ConnectNo, WORD start, WORD inum, float* pdata);

SMC_API short __stdcall smc_set_modbus_4x_int(WORD ConnectNo, WORD start, WORD inum, const int* pdata);
SMC_API short __stdcall smc_get_modbus_4x_int(WORD ConnectNo, WORD start, WORD inum, int* pdata);

SMC_API short __stdcall convert_to_float(WORD *pWORD, float* value);
SMC_API short __stdcall convert_to_int(WORD *pWORD, int* value);

/*********************************************************************************************************
*********************************************************************************************************/
SMC_API short __stdcall smc_set_persistent_reg(WORD ConnectNo, DWORD start, DWORD inum, const char* pdata);
SMC_API short __stdcall smc_get_persistent_reg(WORD ConnectNo, DWORD start, DWORD inum, char* pdata);


SMC_API short __stdcall smc_set_persistent_reg_byte(WORD ConnectNo, DWORD start, DWORD inum, const char* pdata);
SMC_API short __stdcall smc_get_persistent_reg_byte(WORD ConnectNo, DWORD start, DWORD inum, char* pdata);
SMC_API short __stdcall smc_set_persistent_reg_float(WORD ConnectNo, DWORD start, DWORD inum, const float* pdata);
SMC_API short __stdcall smc_get_persistent_reg_float(WORD ConnectNo, DWORD start, DWORD inum, float* pdata);
SMC_API short __stdcall smc_set_persistent_reg_int(WORD ConnectNo, DWORD start, DWORD inum, const int* pdata);
SMC_API short __stdcall smc_get_persistent_reg_int(WORD ConnectNo, DWORD start, DWORD inum, int* pdata);
SMC_API short __stdcall smc_set_persistent_reg_short(WORD ConnectNo, DWORD start, DWORD inum, const short* pdata);
SMC_API short __stdcall smc_get_persistent_reg_short(WORD ConnectNo, DWORD start, DWORD inum, short* pdata);
/*********************************************************************************************************
Basic
*********************************************************************************************************/
SMC_API short __stdcall smc_read_array(WORD ConnectNo ,const char* name,uint32 index,int64* var,int32 *num);
SMC_API short __stdcall smc_modify_array(WORD ConnectNo ,const char* name,uint32 index,int64* var,int32 num);
SMC_API short __stdcall smc_read_var(WORD ConnectNo,const char* varstring, int64* var,int32 *num);
SMC_API short __stdcall smc_modify_var(WORD ConnectNo,const char*varstring, int64* var,int32 varnum);
SMC_API short __stdcall smc_write_array(WORD ConnectNo ,const char* name,uint32 startindex,int64* var,int32 num);

SMC_API short __stdcall smc_read_array_ex(WORD ConnectNo ,const char* name,uint32 index,double* var,int32 *num);
SMC_API short __stdcall smc_modify_array_ex(WORD ConnectNo ,const char* name,uint32 index,double* var,int32 num);
SMC_API short __stdcall smc_read_var_ex(WORD ConnectNo,const char* varstring, double* var,int32 *num);
SMC_API short __stdcall smc_modify_var_ex(WORD ConnectNo,const char*varstring, double* var,int32 varnum);
SMC_API short __stdcall smc_write_array_ex(WORD ConnectNo ,const char* name,uint32 startindex,double* var,int32 num);

SMC_API short __stdcall smc_get_stringtype(WORD ConnectNo,const char* varstring,int32* m_Type,int32* num);
SMC_API short __stdcall smc_basic_delete_file(WORD ConnectNo);
SMC_API short __stdcall smc_basic_run(WORD ConnectNo);
SMC_API short __stdcall smc_basic_stop(WORD ConnectNo);
SMC_API short __stdcall smc_basic_pause(WORD ConnectNo);
SMC_API short __stdcall smc_basic_step_run(WORD ConnectNo);
SMC_API short __stdcall smc_basic_step_over(WORD ConnectNo);
SMC_API short __stdcall smc_basic_continue_run(WORD ConnectNo);
SMC_API short __stdcall smc_basic_state(WORD ConnectNo, WORD* State);
SMC_API short __stdcall smc_basic_current_line(WORD ConnectNo, uint32* line);
SMC_API short __stdcall smc_basic_break_info(WORD ConnectNo, uint32* line,uint32 linenum);
SMC_API short __stdcall smc_basic_message(WORD ConnectNo, char * pbuff, uint32 uimax, uint32 *puiread);
SMC_API short __stdcall smc_basic_command(WORD ConnectNo, const char* pszCommand, char* psResponse, uint32 uiResponseLength);
SMC_API short __stdcall smc_basic_get_sub_program_line(WORD ConnectNo,const char *SubName,char *pLineString);
/*********************************************************************************************************
*********************************************************************************************************/
SMC_API short __stdcall smc_gcode_check_file(WORD ConnectNo, const char* pfilenameinControl, uint8 *pbIfExist, uint32 *pFileSize);
SMC_API short __stdcall smc_gcode_delete_file(WORD ConnectNo, const char* pfilenameinControl);
SMC_API short __stdcall smc_gcode_clear_file(WORD ConnectNo);
SMC_API short __stdcall smc_gcode_get_first_file(WORD ConnectNo,char* pfilenameinControl,uint32* pFileSize);
SMC_API short __stdcall smc_gcode_get_next_file(WORD ConnectNo,char* pfilenameinControl,uint32* pFileSize);
SMC_API short __stdcall smc_gcode_start(WORD ConnectNo);
SMC_API short __stdcall smc_gcode_stop(WORD ConnectNo);
SMC_API short __stdcall smc_gcode_pause(WORD ConnectNo);
SMC_API short __stdcall smc_gcode_state(WORD ConnectNo,  WORD* State);
SMC_API short __stdcall smc_gcode_set_current_file(WORD ConnectNo,const char* pFileName);
SMC_API short __stdcall smc_gcode_get_current_file(WORD ConnectNo,char* pfilenameinControl,WORD *fileid);
SMC_API short __stdcall smc_gcode_current_line(WORD ConnectNo,uint32* line,char* pCurLine);
SMC_API short __stdcall smc_gcode_get_current_line(WORD ConnectNo,uint32* line,char* pCurLine);
SMC_API short __stdcall smc_gcode_get_line(WORD ConnectNo,uint32 line,char* pCurLine);
SMC_API short __stdcall smc_gcode_check_file_id(WORD ConnectNo, WORD fileid, char *pFileName,uint32 *pFileSize, uint32 *pTotalLine);
SMC_API short __stdcall smc_gcode_check_file_name(WORD ConnectNo,const char *pFileName, WORD *fileid, uint32 *pFileSize, uint32 *pTotalLine);
SMC_API short __stdcall smc_gcode_get_file_profile(WORD ConnectNo,uint32* maxfilenum,uint32* maxfilesize,uint32* savedfilenum);

SMC_API short __stdcall smc_gcode_add_line(WORD ConnectNo,const char *strline);
SMC_API short __stdcall smc_gcode_add_line_array(WORD ConnectNo,int arraysize,const float* linearray);
SMC_API short __stdcall smc_gcode_insert_line(WORD ConnectNo,int lineno,const char *strline);
SMC_API short __stdcall smc_gcode_insert_line_array(WORD ConnectNo,int lineno,int arraysize,const float *linearray);
SMC_API short __stdcall smc_gcode_modify_line(WORD ConnectNo,int lineno,const char *strline);
SMC_API short __stdcall smc_gcode_modify_line_array(WORD ConnectNo,int lineno,int arraysize,const float *linearray);
SMC_API short __stdcall smc_gcode_delete_line(WORD ConnectNo,int lineno);
SMC_API short __stdcall smc_gcode_get_line_array(WORD ConnectNo,int lineno,int *arraysize,float *linearray);

SMC_API short __stdcall smc_gcode_create_file(WORD ConnectNo,const char *FileName);
SMC_API short __stdcall smc_gcode_save_file(WORD ConnectNo,const char *FileName);
SMC_API short __stdcall smc_gcode_copy_file(WORD ConnectNo,const char* strFileName,const char* newFileName);
SMC_API short __stdcall smc_gcode_rename_file(WORD ConnectNo,const char* strFileName,const char* newFileName);
SMC_API short __stdcall smc_gcode_delete_fileid(WORD ConnectNo,int fileid);

SMC_API short __stdcall smc_gcode_get_fileid(WORD ConnectNo,uint32 fileid,char *pFileName,uint32 *filesize);

SMC_API short __stdcall smc_gcode_set_step_state(WORD ConnectNo,WORD state);
SMC_API short __stdcall smc_gcode_get_step_state(WORD ConnectNo,WORD *state);
SMC_API short __stdcall smc_gcode_stop_reason(WORD ConnectNo,WORD *stop_reason);

SMC_API short __stdcall smc_gcode_set_speed_profile(WORD ConnectNo,int  fileid, float PathStartSpeed,float PathSpeed,float PathTacc,float MoveStartSpeed,float MoveSpeed,float MoveTacc);
SMC_API short __stdcall smc_gcode_get_speed_profile(WORD ConnectNo,int  fileid, float *PathStartSpeed,float *PathSpeed,float *PathTacc,float *MoveStartSpeed,float *MoveSpeed,float *MoveTacc);
SMC_API short __stdcall smc_gcode_set_start_position(WORD ConnectNo,int  fileid, float *StartPos);
SMC_API short __stdcall smc_gcode_get_start_position(WORD ConnectNo,int  fileid, float *StartPos);

SMC_API short __stdcall smc_glue_set_process_profile(WORD ConnectNo,int  fileid, float OpenAhead, float OpenDelay,float CloseDelay,float CloseAhead,float DrawDist,float DrawHigh,float UpHigh);
SMC_API short __stdcall smc_glue_get_process_profile(WORD ConnectNo,int  fileid, float *OpenAhead, float *OpenDelay,float *CloseDelay,float *CloseAhead,float *DrawDist,float *DrawHigh,float *UpHigh);
SMC_API short __stdcall smc_glue_set_speed_profile(WORD ConnectNo,int  fileid, float DrawStartSpeed,float DrawSpeed,float DrawTacc,float UpStartSpeed,float UpSpeed,float UpTacc);
SMC_API short __stdcall smc_glue_get_speed_profile(WORD ConnectNo,int  fileid, float *DrawStartSpeed,float *DrawSpeed,float *DrawTacc,float *UpStartSpeed,float *UpSpeed,float *UpTacc);
SMC_API short __stdcall smc_glue_set_drop_profile(WORD ConnectNo,WORD enable, float WaitTime,float DropTime,float IntervalTime,float *DropPos);
SMC_API short __stdcall smc_glue_get_drop_profile(WORD ConnectNo,WORD *enable, float *WaitTime,float *DropTime,float *IntervalTime,float *DropPos);
SMC_API short __stdcall smc_sys_set_jog_profile(WORD ConnectNo,WORD SpeedMode,float *JogSpeed,float *JogTacc,float JogDist);
SMC_API short __stdcall smc_sys_get_jog_profile(WORD ConnectNo,WORD SpeedMode,float *JogSpeed,float *JogTacc,float *JogDist);


SMC_API short __stdcall smc_read_trace_data(WORD ConnectNo,WORD axis,long bufsize,double *time,double *pos,double *vel,double *acc,long *recv_num);
SMC_API short __stdcall smc_pmove_sin_unit(WORD ConnectNo,WORD axis,double Dist,WORD posi_mode,double MaxVel,double MaxAcc);


SMC_API short __stdcall smc_trace_set_enable(WORD ConnectNo,WORD axis,WORD enable);

SMC_API short __stdcall smc_trace_set_source(WORD ConnectNo,WORD source);
SMC_API short __stdcall smc_trace_start(WORD ConnectNo,WORD AxisNum,WORD *AxisList);

SMC_API short __stdcall smc_trace_start_ex(WORD ConnectNo,WORD AxisNum,WORD *AxisList);
SMC_API short __stdcall smc_trace_stop(WORD ConnectNo);



SMC_API short __stdcall smc_config_can_profile(WORD ConnectNo,const char *pCfgFile,const char *pFomatFile);



SMC_API short __stdcall nmcs_reset_canopen(WORD ConnectNo);
SMC_API short __stdcall nmcs_reset_rtex(WORD ConnectNo);
SMC_API short __stdcall nmcs_reset_etc(WORD ConnectNo);

SMC_API short __stdcall nmcs_get_total_axes(WORD ConnectNo,DWORD *TotalAxis);
SMC_API short __stdcall nmcs_get_total_ionum(WORD ConnectNo,WORD *TotalIn,WORD *TotalOut);
SMC_API short __stdcall nmcs_get_total_adcnum(WORD ConnectNo,WORD *TotalIn,WORD *TotalOut);

SMC_API short __stdcall nmcs_stop_etc(WORD ConnectNo,WORD* ETCState);
SMC_API short __stdcall nmcs_set_controller_workmode(WORD ConnectNo,WORD controller_mode);
SMC_API short __stdcall nmcs_get_controller_workmode(WORD ConnectNo,WORD* controller_mode);


SMC_API short __stdcall nmcs_set_manager_para(WORD ConnectNo,WORD PortNum,DWORD Baudrate,WORD ManagerID);
SMC_API short __stdcall nmcs_get_manager_para(WORD ConnectNo,WORD PortNum,DWORD *Baudrate,WORD *ManagerID);
SMC_API short __stdcall nmcs_set_manager_od(WORD ConnectNo,WORD PortNum, WORD Index,WORD SubIndex,WORD ValLength,DWORD Value);
SMC_API short __stdcall nmcs_get_manager_od(WORD ConnectNo,WORD PortNum, WORD Index,WORD SubIndex,WORD ValLength,DWORD *Value);
SMC_API short __stdcall nmcs_set_node_od(WORD ConnectNo,WORD PortNum,WORD NodeNum, WORD Index,WORD SubIndex,WORD ValLength,DWORD Value);
SMC_API short __stdcall nmcs_get_node_od(WORD ConnectNo,WORD PortNum,WORD NodeNum, WORD Index,WORD SubIndex,WORD ValLength,DWORD* Value);

SMC_API short __stdcall nmcs_reset_to_factory(WORD ConnectNo,WORD PortNum,WORD NodeNum);
SMC_API short __stdcall nmcs_write_to_pci(WORD ConnectNo,WORD PortNum,WORD NodeNum);


SMC_API short __stdcall nmcs_set_axis_enable(WORD ConnectNo ,WORD axis);
SMC_API short __stdcall nmcs_set_axis_disable(WORD ConnectNo,WORD axis);

SMC_API short __stdcall nmcs_set_alarm_clear(WORD ConnectNo,WORD PortNum,WORD NodeNum);
SMC_API short __stdcall nmcs_clear_alarm_fieldbus(WORD ConnectNo,WORD PortNum);

SMC_API short __stdcall nmcs_set_axis_contrlword(WORD ConnectNo,WORD Axis,long Contrlword);
SMC_API short __stdcall nmcs_get_axis_contrlword(WORD ConnectNo,WORD Axis,long *Contrlword);
SMC_API short __stdcall nmcs_set_axis_contrlmode(WORD ConnectNo,WORD Axis,long Contrlmode);
SMC_API short __stdcall nmcs_get_axis_contrlmode(WORD ConnectNo,WORD Axis,long *Contrlmode);

SMC_API short __stdcall nmcs_get_errcode(WORD ConnectNo,WORD PortNum,DWORD *Errcode);
SMC_API short __stdcall nmcs_get_card_errcode(WORD ConnectNo,DWORD *Errcode);
SMC_API short __stdcall nmcs_get_axis_errcode(WORD ConnectNo,WORD axis,DWORD *Errcode);
SMC_API short __stdcall nmcs_clear_errcode(WORD ConnectNo,WORD PortNum);
SMC_API short __stdcall nmcs_clear_card_errcode(WORD ConnectNo);
SMC_API short __stdcall nmcs_clear_axis_errcode(WORD ConnectNo,WORD iaxis);


SMC_API short __stdcall nmcs_get_LostHeartbeat_Nodes(WORD ConnectNo,WORD PortNum,WORD* NodeID,WORD* NodeNum);
SMC_API short __stdcall nmcs_get_EmergeneyMessege_Nodes(WORD ConnectNo,WORD PortNum,DWORD* NodeMsg,WORD* MsgNum);
SMC_API short __stdcall nmcs_SendNmtCommand(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD NmtCommand);

SMC_API short __stdcall nmcs_syn_move_unit(WORD ConnectNo,WORD AxisNum,WORD* AxisList,double* Position,WORD* PosiMode);

SMC_API short __stdcall nmcs_axis_io_status(WORD ConnectNo,WORD axis);	
SMC_API DWORD __stdcall nmcs_get_axis_io_out(WORD ConnectNo,WORD axis);
SMC_API short __stdcall nmcs_set_axis_io_out(WORD ConnectNo,WORD axis,DWORD  iostate);
SMC_API DWORD __stdcall nmcs_get_axis_io_in(WORD ConnectNo,WORD axis);

SMC_API short __stdcall nmcs_set_master_para(WORD ConnectNo,WORD PortNum,WORD Baudrate,DWORD NodeCnt,WORD MasterId);

SMC_API short __stdcall nmcs_get_master_para(WORD ConnectNo,WORD PortNum,WORD *Baudrate,DWORD *NodeCnt,WORD *MasterId);

SMC_API short __stdcall nmcs_write_outbit(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD IoBit,WORD IoValue);

SMC_API short __stdcall nmcs_read_outbit(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD IoBit,WORD *IoValue);

SMC_API short __stdcall nmcs_read_inbit(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD IoBit,WORD *IoValue);

SMC_API short __stdcall nmcs_write_outport(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD PortNo,DWORD IoValue);

SMC_API short __stdcall nmcs_read_outport(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD PortNo,DWORD *IoValue);

SMC_API short __stdcall nmcs_read_inport(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD PortNo,DWORD *IoValue);



SMC_API short __stdcall nmcs_write_outbit_extern(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD IoBit,WORD IoValue);

SMC_API short __stdcall nmcs_read_outbit_extern(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD IoBit,WORD *IoValue);

SMC_API short __stdcall nmcs_read_inbit_extern(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD IoBit,WORD *IoValue);

SMC_API short __stdcall nmcs_write_outport_extern(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD PortNo,DWORD IoValue);

SMC_API short __stdcall nmcs_read_outport_extern(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD PortNo,DWORD *IoValue);

SMC_API short __stdcall nmcs_read_inport_extern(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD PortNo,DWORD *IoValue);

SMC_API short __stdcall nmcs_set_da_output(WORD ConnectNo,WORD PortNum,WORD NoteID,WORD channel,DWORD Value);
SMC_API short __stdcall nmcs_get_da_output(WORD ConnectNo,WORD PortNum,WORD NoteID,WORD channel,DWORD *Value);
SMC_API short __stdcall nmcs_get_ad_input(WORD ConnectNo,WORD PortNum,WORD NoteID,WORD channel,DWORD *Value);

SMC_API short __stdcall nmcs_get_slave_nodes(WORD ConnectNo,WORD PortNum,WORD BaudRate,WORD* NodeId,WORD* NodeNum);

SMC_API short __stdcall nmcs_set_cycletime(WORD ConnectNo,WORD PortNum,DWORD CycleTime);
SMC_API short __stdcall nmcs_get_cycletime(WORD ConnectNo,WORD PortNum,DWORD* CycleTime);

SMC_API short __stdcall nmcs_get_consume_time_fieldbus(WORD ConnectNo,WORD PortNum,DWORD* Average_time, DWORD* Max_time,uint64* Cycles);
SMC_API short __stdcall nmcs_get_perline_time(WORD ConnectNo,WORD index,DWORD* Average_time, DWORD* Max_time,uint64* Cycles);
SMC_API short __stdcall nmcs_clear_consume_time_fieldbus(WORD ConnectNo,WORD PortNum);

SMC_API short __stdcall nmcs_get_axis_type(WORD ConnectNo,WORD axis, WORD* Axis_Type);
SMC_API short __stdcall nmcs_get_axis_node_address(WORD ConnectNo,WORD axis, WORD* SlaveAddr,WORD* Sub_SlaveAddr);
SMC_API short __stdcall nmcs_get_axis_state_machine(WORD ConnectNo,WORD axis, WORD* Axis_StateMachine);
SMC_API short __stdcall nmcs_write_rxpdo_extra(WORD ConnectNo,WORD PortNum,WORD address,WORD DataLen,DWORD Value);
SMC_API short __stdcall nmcs_read_rxpdo_extra(WORD ConnectNo,WORD PortNum,WORD address,WORD DataLen,DWORD* Value);
SMC_API short __stdcall nmcs_read_txpdo_extra(WORD ConnectNo,WORD PortNum,WORD address,WORD DataLen,DWORD* Value);


SMC_API short __stdcall nmc_set_home_profile(WORD ConnectNo ,WORD axis,WORD home_mode,double High_Vel, double Low_Vel,double Tacc,double Tdec ,double offsetpos);
SMC_API short __stdcall nmc_get_home_profile(WORD ConnectNo ,WORD axis,WORD* home_mode,double* High_Vel, double* Low_Vel,double* Tacc,double* Tdec ,double* offsetpos);
SMC_API short __stdcall nmc_home_move(WORD ConnectNo,WORD axis);


SMC_API short __stdcall nmcs_start_connect(WORD ConnectNo,WORD chan,WORD*info,WORD* len);
SMC_API short __stdcall nmcs_get_vendor_info(WORD ConnectNo,WORD axis,char* info,WORD* len);
SMC_API short __stdcall nmcs_get_slave_type_info(WORD ConnectNo,WORD axis,char* info,WORD* len);
SMC_API short __stdcall nmcs_get_slave_name_info(WORD ConnectNo,WORD axis,char* info,WORD* len);
SMC_API short __stdcall nmcs_get_slave_version_info(WORD ConnectNo,WORD axis,char* info,WORD* len);

SMC_API short __stdcall nmcs_write_parameter(WORD ConnectNo,WORD axis,WORD index, WORD subindex,DWORD para_data);
SMC_API short __stdcall nmcs_write_slave_eeprom(WORD ConnectNo,WORD axis);
SMC_API short __stdcall nmcs_read_parameter(WORD ConnectNo,WORD axis,WORD index, WORD subindex,DWORD* para_data);
SMC_API short __stdcall nmcs_read_parameter_attributes(WORD ConnectNo,WORD axis,WORD index, WORD subindex,DWORD* para_data);
SMC_API short __stdcall nmcs_set_cmdcycletime(WORD ConnectNo,WORD PortNum,DWORD cmdtime);
SMC_API short __stdcall nmcs_get_cmdcycletime(WORD ConnectNo,WORD PortNum,DWORD* cmdtime);
SMC_API short __stdcall nmcs_start_log(WORD ConnectNo,WORD chan,WORD node, WORD Ifenable);
SMC_API short __stdcall nmcs_get_log(WORD ConnectNo,WORD chan,WORD node, DWORD* data);
SMC_API short __stdcall nmcs_config_atuo_log(WORD ConnectNo,WORD ifenable,WORD dir,WORD byte_index,WORD mask,WORD condition,DWORD counter);
SMC_API short __stdcall nmcs_get_log_state(WORD ConnectNo,WORD chan, DWORD* state);
SMC_API short __stdcall nmcs_driver_reset(WORD ConnectNo,WORD axis);
SMC_API short __stdcall nmc_set_manager_para(WORD ConnectNo,WORD PortNum,DWORD Baudrate,WORD ManagerID);
SMC_API short __stdcall nmc_get_manager_para(WORD ConnectNo,WORD PortNum,DWORD *Baudrate,WORD *ManagerID);
SMC_API short __stdcall nmc_set_manager_od(WORD ConnectNo,WORD PortNum, WORD Index,WORD SubIndex,WORD ValLength,DWORD Value);
SMC_API short __stdcall nmc_get_manager_od(WORD ConnectNo,WORD PortNum, WORD Index,WORD SubIndex,WORD ValLength,DWORD *Value);
SMC_API short __stdcall nmc_set_node_od(WORD ConnectNo,WORD PortNum,WORD NodeNum, WORD Index,WORD SubIndex,WORD ValLength,DWORD Value);
SMC_API short __stdcall nmc_get_node_od(WORD ConnectNo,WORD PortNum,WORD NodeNum, WORD Index,WORD SubIndex,WORD ValLength,DWORD* Value);


SMC_API short __stdcall nmc_reset_to_factory(WORD ConnectNo,WORD PortNum,WORD NodeNum);
SMC_API short __stdcall nmc_write_to_pci(WORD ConnectNo,WORD PortNum,WORD NodeNum);
    
SMC_API short __stdcall nmc_set_axis_enable(WORD ConnectNo,WORD PortNum,WORD NodeNum);
SMC_API short __stdcall nmcs_set_axis_disable(WORD ConnectNo,WORD axis);
SMC_API short __stdcall nmc_set_alarm_clear(WORD ConnectNo,WORD PortNum,WORD NodeNum);

SMC_API short __stdcall nmc_set_axis_contrlword(WORD ConnectNo,WORD Axis,long Contrlword);
SMC_API short __stdcall nmc_get_axis_contrlword(WORD ConnectNo,WORD Axis,long *Contrlword);
SMC_API short __stdcall nmc_set_axis_contrlmode(WORD ConnectNo,WORD Axis,long Contrlmode);
SMC_API short __stdcall nmc_get_axis_contrlmode(WORD ConnectNo,WORD Axis,long *Contrlmode);

SMC_API short __stdcall nmc_get_errcode(WORD ConnectNo,WORD PortNum,DWORD *Errcode);
SMC_API short __stdcall nmc_get_card_errcode(WORD ConnectNo,DWORD *Errcode);
SMC_API short __stdcall nmc_get_axis_errcode(WORD ConnectNo,WORD axis,DWORD *Errcode);
SMC_API short __stdcall nmc_clear_errcode(WORD ConnectNo,WORD iaxis);
SMC_API short __stdcall nmc_clear_card_errcode(WORD ConnectNo);
SMC_API short __stdcall nmc_clear_axis_errcode(WORD ConnectNo,WORD iaxis);

SMC_API short __stdcall nmc_get_total_axes(WORD ConnectNo,DWORD *TotalAxis);
SMC_API short __stdcall nmc_get_total_ionum(WORD ConnectNo,WORD *TotalIn,WORD *TotalOut);
SMC_API short __stdcall nmc_get_total_adcnum(WORD ConnectNo,WORD *TotalIn,WORD *TotalOut);
SMC_API short __stdcall nmc_get_LostHeartbeat_Nodes(WORD ConnectNo,WORD PortNum,WORD* NodeID,WORD* NodeNum);
SMC_API short __stdcall nmc_get_EmergeneyMessege_Nodes(WORD ConnectNo,WORD PortNum,DWORD* NodeMsg,WORD* MsgNum);
SMC_API short __stdcall nmc_SendNmtCommand(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD NmtCommand);
SMC_API short __stdcall nmc_syn_move_unit(WORD ConnectNo,WORD AxisNum,WORD* AxisList,double* Position,WORD* PosiMode);
    
SMC_API short __stdcall nmc_axis_io_status(WORD ConnectNo,WORD axis);	//��ȡָ�����й��˶��źŵ�״̬
    

SMC_API short __stdcall nmc_set_master_para(WORD ConnectNo,WORD PortNum,WORD Baudrate,DWORD NodeCnt,WORD MasterId);
SMC_API short __stdcall nmc_get_master_para(WORD ConnectNo,WORD PortNum,WORD *Baudrate,DWORD *NodeCnt,WORD *MasterId);
SMC_API short __stdcall nmc_write_outbit(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD IoBit,WORD IoValue);
SMC_API short __stdcall nmc_read_outbit(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD IoBit,WORD *IoValue);
SMC_API short __stdcall nmc_read_inbit(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD IoBit,WORD *IoValue);
SMC_API short __stdcall nmc_write_outport(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD PortNo,DWORD IoValue);
SMC_API short __stdcall nmc_read_outport(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD PortNo,DWORD *IoValue);
SMC_API short __stdcall nmc_read_inport(WORD ConnectNo,WORD PortNum,WORD NodeID,WORD PortNo,DWORD *IoValue);
SMC_API short __stdcall nmc_set_da_output(WORD ConnectNo,WORD PortNum,WORD NoteID,WORD channel,DWORD Value);
SMC_API short __stdcall nmc_get_da_output(WORD ConnectNo,WORD PortNum,WORD NoteID,WORD channel,DWORD *Value);
SMC_API short __stdcall nmc_get_ad_input(WORD ConnectNo,WORD PortNum,WORD NoteID,WORD channel,DWORD *Value);
SMC_API short __stdcall nmc_get_slave_nodes(WORD ConnectNo,WORD PortNum,WORD BaudRate,WORD* NodeId,WORD* NodeNum);

SMC_API short __stdcall smc_check_done_crd(WORD ConnectNo,WORD Crd);
SMC_API short __stdcall smc_stop_crd(WORD ConnectNo,WORD Crd,WORD stop_mode);

SMC_API short __stdcall smc_SetLtcOutMode(WORD ConnectNo,WORD axis,WORD enable,WORD bitno);
SMC_API short __stdcall smc_GetLtcOutMode(WORD ConnectNo,WORD axis,WORD *enable,WORD* bitno);
SMC_API short __stdcall smc_glue_set_profile(WORD ConnectNo,WORD glue,WORD io,WORD on_off,double* Offset,double* dist,double* time);
SMC_API short __stdcall smc_glue_get_profile(WORD ConnectNo,WORD glue,WORD* io,WORD* on_off,double* Offset,double* dist,double* time);

SMC_API short __stdcall smc_set_vector_taccdec(WORD ConnectNo,WORD Crd,double Taccdec);

SMC_API short __stdcall smc_set_latch_stop_time(WORD ConnectNo,WORD axis,long time);
SMC_API short __stdcall smc_get_latch_stop_time(WORD ConnectNo,WORD axis,long* time);

SMC_API short __stdcall smc_conti_set_taccdec(WORD ConnectNo,WORD Crd,double Taccdec);
SMC_API short __stdcall smc_conti_set_speed_unit(WORD ConnectNo,WORD Crd,double Max_vel);
    
    
SMC_API short __stdcall smc_set_cmp_config(WORD ConnectNo,WORD axis, long cmp_pos, WORD cmp_source, WORD cmp_logic);
SMC_API short __stdcall smc_get_cmp_config(WORD ConnectNo,WORD axis, long* cmp_pos, WORD* cmp_source, WORD* cmp_logic);
SMC_API short __stdcall smc_set_cmp_enable(WORD ConnectNo,WORD axis, WORD cmp_enable);
SMC_API short __stdcall smc_get_cmp_enable(WORD ConnectNo,WORD axis, WORD* cmp_enable);

SMC_API short __stdcall smc_get_latch_flag_extern(WORD ConnectNo,WORD axis);
SMC_API double __stdcall smc_get_latch_value_extern_unit(WORD ConnectNo,WORD axis,WORD index);
SMC_API long __stdcall smc_get_latch_value_extern(WORD ConnectNo,WORD axis,WORD index);
SMC_API long __stdcall smc_get_latch_value(WORD ConnectNo,WORD axis);
SMC_API long __stdcall smc_get_encoder(WORD ConnectNo,WORD axis);
SMC_API short __stdcall smc_set_encoder(WORD ConnectNo,WORD axis,long encoder_value);

SMC_API short __stdcall smc_conti_delay_pwm_to_stop(WORD ConnectNo, WORD Crd, WORD pwmno,WORD on_off,double delay_time,double ReverseTime);

SMC_API short __stdcall smc_conti_rectangle_move_unit(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Mark_Pos,long num,WORD rect_mode,WORD posi_mode,long mark);

SMC_API short __stdcall smc_conti_line_unit1(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* pPosList,WORD posi_mode,long mark,double MaxVel,double Taccdec,double Tspara);
SMC_API short __stdcall smc_conti_arc_move_center_unit1(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Cen_Pos,WORD Arc_Dir,long Circle,WORD posi_mode,long mark,double MaxVel,double Taccdec,double Tspara);
SMC_API short __stdcall smc_conti_arc_move_radius_unit1(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double Arc_Radius,WORD Arc_Dir,long Circle,WORD posi_mode,long mark,double MaxVel,double Taccdec,double Tspara);
SMC_API short __stdcall smc_conti_arc_move_3points_unit1(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Mid_Pos,long Circle,WORD posi_mode,long mark,double MaxVel,double Taccdec,double Tspara);

SMC_API short __stdcall smc_conti_set_involute_mode(WORD ConnectNo,WORD Crd,WORD mode);
SMC_API short __stdcall smc_conti_get_involute_mode(WORD ConnectNo,WORD Crd,WORD* mode);
SMC_API short __stdcall smc_conti_set_profile_unit(WORD ConnectNo,WORD Crd,double Min_Vel,double Max_vel,double Tacc,double Tdec,double Stop_Vel);
SMC_API short __stdcall smc_conti_set_s_profile(WORD ConnectNo,WORD Crd,WORD s_mode,double s_para);
SMC_API short __stdcall smc_conti_get_s_profile(WORD ConnectNo,WORD Crd,WORD s_mode,double* s_para);

SMC_API short __stdcall smc_set_softlimit(WORD ConnectNo,WORD axis,WORD enable, WORD source_sel,WORD SL_action, long N_limit,long P_limit);
SMC_API short __stdcall smc_get_softlimit(WORD ConnectNo,WORD axis,WORD *enable, WORD *source_sel,WORD *SL_action,long *N_limit,long *P_limit);

SMC_API short __stdcall smc_set_backlash(WORD ConnectNo,WORD axis,long backlash);
SMC_API short __stdcall smc_get_backlash(WORD ConnectNo,WORD axis,long *backlash);
SMC_API short __stdcall smc_set_profile(WORD ConnectNo,WORD axis,double Min_Vel,double Max_Vel,double Tacc,double Tdec,double stop_vel);	
SMC_API short __stdcall smc_get_profile(WORD ConnectNo,WORD axis,double *Min_Vel,double *Max_Vel,double *Tacc,double *Tdec,double *stop_vel);	
SMC_API short __stdcall smc_pmove(WORD ConnectNo,WORD axis,long dist,WORD posi_mode);	

SMC_API short __stdcall smc_reset_target_position(WORD ConnectNo,WORD axis,long dist,WORD posi_mode);	
SMC_API short __stdcall smc_change_speed(WORD ConnectNo,WORD axis,double Curr_Vel,double Taccdec);	
SMC_API short __stdcall smc_update_target_position(WORD ConnectNo,WORD axis,long dist,WORD posi_mode);	

SMC_API short __stdcall smc_set_special_input_filter(WORD ConnectNo,double Filter);
SMC_API short __stdcall smc_download_configfile(WORD ConnectNo,const char *FileName);
SMC_API short __stdcall smc_download_firmware(WORD ConnectNo,const char *FileName);
    

SMC_API short __stdcall smc_set_handwheel_channel(WORD ConnectNo,WORD index);
SMC_API short __stdcall smc_get_handwheel_channel(WORD ConnectNo,WORD* index);


SMC_API short __stdcall smc_PvtTable(WORD ConnectNo,WORD iaxis,DWORD count,double *pTime,long *pPos,double *pVel);
SMC_API short __stdcall smc_PtsTable(WORD ConnectNo,WORD iaxis,DWORD count,double *pTime,long *pPos,double *pPercent);
SMC_API short __stdcall smc_PvtsTable(WORD ConnectNo,WORD iaxis,DWORD count,double *pTime,long *pPos,double velBegin,double velEnd);
SMC_API short __stdcall smc_PttTable(WORD ConnectNo,WORD iaxis,DWORD count,double *pTime,long *pPos);
SMC_API short __stdcall smc_PvtMove(WORD ConnectNo,WORD AxisNum,WORD* AxisList);

SMC_API short __stdcall smc_set_vector_profile_multicoor(WORD ConnectNo,WORD Crd, double Min_Vel,double Max_Vel,double Tacc,double Tdec,double Stop_Vel);
SMC_API short __stdcall smc_get_vector_profile_multicoor(WORD ConnectNo,WORD Crd, double* Min_Vel,double* Max_Vel,double* Tacc,double* Tdec,double* Stop_Vel);

SMC_API short __stdcall smc_set_vector_s_profile_multicoor(WORD ConnectNo,WORD Crd,WORD s_mode,double s_para);	
SMC_API short __stdcall smc_get_vector_s_profile_multicoor(WORD ConnectNo,WORD Crd,WORD s_mode,double *s_para);	

SMC_API short __stdcall smc_line_multicoor(WORD ConnectNo,WORD Crd,WORD axisNum,WORD *axisList,long *DistList,WORD posi_mode);	
SMC_API short __stdcall smc_arc_move_multicoor(WORD ConnectNo,WORD Crd,WORD *AxisList,long *Target_Pos,long *Cen_Pos,WORD Arc_Dir,WORD posi_mode);
    

SMC_API long __stdcall smc_get_homelatch_value(WORD ConnectNo,WORD axis);

SMC_API short __stdcall smc_set_erc_mode(WORD ConnectNo,WORD axis,WORD enable,WORD erc_logic,WORD erc_width,WORD erc_off_time);	
SMC_API short __stdcall smc_get_erc_mode(WORD ConnectNo,WORD axis,WORD *enable,WORD *erc_logic, WORD *erc_width,WORD *erc_off_time);	

SMC_API short __stdcall smc_write_sevrst_pin(WORD ConnectNo,WORD axis,WORD on_off); 	
SMC_API short __stdcall smc_read_sevrst_pin(WORD ConnectNo,WORD axis); 

SMC_API short __stdcall smc_set_dstp_mode(WORD ConnectNo,WORD axis,WORD enable,WORD logic,DWORD time);
SMC_API short __stdcall smc_get_dstp_mode(WORD ConnectNo,WORD axis,WORD *enable,WORD *logic,DWORD *time);
SMC_API short __stdcall smc_set_dstp_time(WORD ConnectNo,WORD axis,DWORD time);
SMC_API short __stdcall smc_get_dstp_time(WORD ConnectNo,WORD axis,DWORD *time);

SMC_API short __stdcall smc_set_io_dstp_bitno(WORD ConnectNo,WORD axis,WORD bitno,double filter); 	
SMC_API short __stdcall smc_get_io_dstp_bitno(WORD ConnectNo,WORD axis,WORD *bitno,double* filter);
    
    
SMC_API short __stdcall smc_set_dec_stop_dist(WORD ConnectNo,WORD axis,long dist);
SMC_API short __stdcall smc_get_dec_stop_dist(WORD ConnectNo,WORD axis,long *dist);

SMC_API short __stdcall  dmc_check_done_pos(WORD ConnectNo,WORD axis,WORD posi_mode);
SMC_API short __stdcall  dmc_set_factor_error(WORD ConnectNo,WORD axis,double factor,long error);
SMC_API short __stdcall  dmc_set_factor(WORD ConnectNo,WORD axis,double factor);
SMC_API short __stdcall  dmc_set_error(WORD ConnectNo,WORD axis,long error);
SMC_API short __stdcall  dmc_get_factor_error(WORD ConnectNo,WORD axis,double* factor,long* error);
SMC_API short __stdcall  dmc_check_success_pulse(WORD ConnectNo,WORD axis);
SMC_API short __stdcall  dmc_check_success_encoder(WORD ConnectNo,WORD axis);

SMC_API double __stdcall smc_read_current_speed(WORD ConnectNo,WORD axis);	
SMC_API short __stdcall smc_set_position(WORD ConnectNo,WORD axis,long current_position);	
SMC_API long __stdcall smc_get_position(WORD ConnectNo,WORD axis);	
SMC_API long __stdcall smc_get_target_position(WORD ConnectNo,WORD axis);	
SMC_API double __stdcall smc_read_vector_speed(WORD ConnectNo);	

SMC_API short __stdcall smc_read_password(WORD ConnectNo, char * str_sn);
    
SMC_API short __stdcall smc_t_pmove_unit(WORD ConnectNo,WORD axis,double Dist,WORD posi_mode);
SMC_API short __stdcall smc_ex_t_pmove_unit(WORD ConnectNo,WORD axis,double Dist,WORD posi_mode);
SMC_API short __stdcall smc_s_pmove_unit(WORD ConnectNo,WORD axis,double Dist,WORD posi_mode);
SMC_API short __stdcall smc_ex_s_pmove_unit(WORD ConnectNo,WORD axis,double Dist,WORD posi_mode);
    

SMC_API short __stdcall smc_calculate_arclength_center(double* start_pos,double *target_pos,double *cen_pos, WORD arc_dir,double circle,double* ArcLength);
SMC_API short __stdcall smc_calculate_arclength_3point(double *start_pos,double *mid_pos, double *end_pos,double circle,double* ArcLength);
SMC_API short __stdcall smc_calculate_arclength_radius(double* start_pos,double *target_pos,double arc_radius, WORD arc_dir,double circle,double* ArcLength);
SMC_API short __stdcall smc_rectangle_move_unit(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Target_Pos,double* Mark_Pos,long num,WORD rect_mode,WORD posi_mode);

SMC_API short __stdcall smc_line_unit1(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double* Dist,WORD posi_mode,double MaxVel,double Taccdec,double Tspara);
SMC_API short __stdcall smc_arc_move_center_unit1(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Cen_Pos,WORD Arc_Dir,long Circle,WORD posi_mode,double MaxVel,double Taccdec,double Tspara);
SMC_API short __stdcall smc_arc_move_radius_unit1(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double Arc_Radius,WORD Arc_Dir,long Circle,WORD posi_mode,double MaxVel,double Taccdec,double Tspara);
SMC_API short __stdcall smc_arc_move_3points_unit1(WORD ConnectNo,WORD Crd,WORD AxisNum,WORD* AxisList,double *Target_Pos,double *Mid_Pos,long Circle,WORD posi_mode,double MaxVel,double Taccdec,double Tspara);


SMC_API short __stdcall smc_set_can_state(WORD ConnectNo,WORD NodeNum,WORD state);
SMC_API short __stdcall smc_get_can_state(WORD ConnectNo,WORD* NodeNum,WORD* state);
SMC_API short __stdcall smc_get_can_errcode(WORD ConnectNo,WORD *Errcode);
SMC_API short __stdcall smc_write_can_outbit(WORD ConnectNo,WORD Node,WORD bitno,WORD on_off);
SMC_API short __stdcall smc_read_can_outbit(WORD ConnectNo,WORD Node,WORD bitno);
SMC_API short __stdcall smc_read_can_inbit(WORD ConnectNo,WORD Node,WORD bitno);
SMC_API short __stdcall smc_write_can_outport(WORD ConnectNo,WORD Node,WORD PortNo,DWORD outport_val);
SMC_API DWORD __stdcall smc_read_can_outport(WORD ConnectNo,WORD Node,WORD PortNo);
SMC_API DWORD __stdcall smc_read_can_inport(WORD ConnectNo,WORD Node,WORD PortNo);
SMC_API short __stdcall smc_compare_add_point(WORD ConnectNo,WORD axis,long pos,WORD dir, WORD action,DWORD actpara); 	
SMC_API short __stdcall smc_compare_get_current_point(WORD ConnectNo,WORD axis,long *pos); 	
SMC_API short __stdcall smc_compare_add_point_extern(WORD ConnectNo,WORD* axis,long* pos,WORD* dir, WORD action,DWORD actpara); 	
SMC_API short __stdcall smc_compare_get_current_point_extern(WORD ConnectNo,long *pos); 	

SMC_API short __stdcall smc_hcmp_add_point(WORD ConnectNo,WORD hcmp, long cmp_pos);
SMC_API short __stdcall smc_hcmp_set_liner(WORD ConnectNo,WORD hcmp, long Increment,long Count);
SMC_API short __stdcall smc_hcmp_get_liner(WORD ConnectNo,WORD hcmp, long* Increment,long* Count);
SMC_API short __stdcall smc_hcmp_get_current_state(WORD ConnectNo,WORD hcmp,long *remained_points,long *current_point,long *runned_points); 
       
   
#ifdef __cplusplus
}
#endif

#endif
