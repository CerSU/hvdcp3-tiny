/*
 * hvdcp3-tiny driver
 *
 * License Terms: GNU General Public License v2
 *
 * Author:
 *	Dejiang Su <dejiang.su@hotmail.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/io.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/power/charger-manager.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/of.h>
#include <linux/thermal.h>
#include <linux/wait.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/kthread.h>

#define AUTO_ALGO_OPTI 0
enum hvdcp3_tiny_st {
	QC3_AUTH_NO_HVDCP3 = 0,
	QC3_AUTH_PREPARE,
	QC3_AUTH_DP_PULSE,
	QC3_AUTH_CONFIRED,
	QC3_AUTH_OPTI_INITIAL_VOL,
	QC3_OPTI_RERUN_AICL,
	QC3_OPTI_NOT_LIMITED,
	QC3_OPTI_DP_PULSE,
	QC3_OPTI_LIMITED,
	QC3_OPTI_DM_PULSE,
	QC3_ST_MAX,
};


struct hvdcp3_limited_info{
	int	battery_present;
	int	usb_present;
	int	is_usb_hvdcp;
	int	parallel_present;
	int	input_current_limited;
	int	parralel_current_limited;
	int	hvdcp_ov;

	int	usb_uv;
	int	battery_uv;
#if AUTO_ALGO_OPTI
	int	main_battery_ua;
	int	parallel_battery_ua;
	int	main_input_ua;
	int	icl_down_cnt;
#endif

	int	max_pulse_allowed;
	int	pulse_cnt;
	int	aicl_now;
};

struct hvdcp3_tiny {
	bool	hvdcp3_opti_algo;
	bool	hvdcp3_allowed;
	enum	hvdcp3_tiny_st	hts;
	wait_queue_head_t	wait;
	int	pulse_cnt;
	int	pulse_opti;

	bool	usb_insert;

	struct device	*dev;

	struct power_supply	*usb_psy;
	struct power_supply	*battery_psy;
	struct power_supply	*parallel_psy;

	struct mutex ls_lock;
	struct hvdcp3_limited_info *hli;
	struct hvdcp3_cmd_list	*cmd;

	struct task_struct	*task;
	struct notifier_block	nb;

	int (*handle)(struct hvdcp3_tiny *q);
};

struct hvdcp3_cmd_list {
	char *cmd;
	int argv;
	int delay_ms;
	int (*call)(struct hvdcp3_tiny *qc3);
};

#define HVDCP_CMD_LIST(_id,_name,_argv,ms)	\
	[_id] = {		\
		.argv = _argv,	\
		.delay_ms = ms,	\
		.call = qc3_##_name,	\
	}

#define DECLARE_HVDCP_ST_OP(cmd)	\
	static int qc3_##cmd(struct hvdcp3_tiny *qc3)

DECLARE_HVDCP_ST_OP(auth_no_hvdcp3);
DECLARE_HVDCP_ST_OP(auth_prepare);
DECLARE_HVDCP_ST_OP(auth_dp_pulse);
DECLARE_HVDCP_ST_OP(auth_confired);
DECLARE_HVDCP_ST_OP(auth_opti_initial_vol);
DECLARE_HVDCP_ST_OP(opti_rerun_aicl);
DECLARE_HVDCP_ST_OP(opti_not_limited);
DECLARE_HVDCP_ST_OP(opti_dp_pulse);
DECLARE_HVDCP_ST_OP(opti_limited);
DECLARE_HVDCP_ST_OP(opti_dm_pulse);

static struct hvdcp3_cmd_list default_vendor_cmd[QC3_ST_MAX] = {
	HVDCP_CMD_LIST(QC3_AUTH_NO_HVDCP3,auth_no_hvdcp3,0,60*1000),
	HVDCP_CMD_LIST(QC3_AUTH_PREPARE,auth_prepare,0,60),
	HVDCP_CMD_LIST(QC3_AUTH_DP_PULSE,auth_dp_pulse,6,100),
	HVDCP_CMD_LIST(QC3_AUTH_CONFIRED,auth_confired,0,60),
	HVDCP_CMD_LIST(QC3_AUTH_OPTI_INITIAL_VOL,auth_opti_initial_vol,0,60),
	HVDCP_CMD_LIST(QC3_OPTI_RERUN_AICL,opti_rerun_aicl,0,60*1000),
	HVDCP_CMD_LIST(QC3_OPTI_NOT_LIMITED,opti_not_limited,0,60*1000),
	HVDCP_CMD_LIST(QC3_OPTI_DP_PULSE,opti_dp_pulse,0,60*1000),
	HVDCP_CMD_LIST(QC3_OPTI_LIMITED,opti_limited,0,60*1000),
	HVDCP_CMD_LIST(QC3_OPTI_DM_PULSE,opti_dm_pulse,0,60*1000),
};

static const int qc3_max_pulse_allowed = 20;
static const int qc3_auth_pulse = 6;

static const int qc3_usb_ov_voltage_uv = 10*1000*1000;
static const int qc3_confirmed_voltage_uv = 6*1000*1000;

static const int qc3_wait_2s = 2*1000;
static const int qc3_wait_60s = 60*1000;

static int hvdcp3_tiny_suspend_prepare(struct device *dev)
{
	return 0;
}

static int hvdcp3_tiny_suspend_suspend(struct device *dev)
{
	return 0;
}

static int hvdcp3_tiny_suspend_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops hvdcp3_tiny_pm = {
	.prepare = hvdcp3_tiny_suspend_prepare,
	.suspend = hvdcp3_tiny_suspend_suspend,
	.resume = hvdcp3_tiny_suspend_resume,
};

static struct of_device_id hvdcp3_tiny_match[] = {
	{
		.compatible = "hvdcp3-tiny",
	},
	{},
};

static const struct platform_device_id hvdcp3_tiny_id[] = {
	{ "hvdcp3-tiny", 0 },
	{ },
};

static int get_int_prop_from_psy(struct power_supply *psy,
			enum power_supply_property prop)
{
	int rc;
	union power_supply_propval ret = {0, };

	if( IS_ERR_OR_NULL(psy))
		return -EINVAL;

	rc = psy->get_property(psy, prop, &ret);

	if(rc < 0) {
		pr_err("Couldn't get psy property rc=%d",rc);
		return -EINVAL;
	}

	return ret.intval;
}

static int set_int_prop_to_psy(struct power_supply *psy,enum power_supply_property prop,int val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if( IS_ERR_OR_NULL(psy))
		return -EINVAL;

	ret.intval = val;
	rc = psy->set_property(psy, prop, &ret);

	if(rc < 0) {
		pr_err("Couldn't set psy %d property rc=%d",prop,rc);
		return -EINVAL;
	}

	return rc;
}

static bool is_hvdcp_type(struct hvdcp3_tiny *q)
{
	union power_supply_propval ret;
	int rc;

	if(IS_ERR_OR_NULL(q->usb_psy))
		return 0;

	rc = q->usb_psy->get_property(q->usb_psy, POWER_SUPPLY_PROP_TYPE,
			&ret);
	if(rc)
		pr_err("Couldn't get usb type\n");

	if(ret.intval == POWER_SUPPLY_TYPE_USB_HVDCP ||
		ret.intval == POWER_SUPPLY_TYPE_USB_HVDCP_3)
		return 1;

	return 0;
}

static bool is_hvdcp_ov(int uv)
{
	return uv >= qc3_usb_ov_voltage_uv ? 1:0;
}

static int hvdcp3_get_limited_info(struct hvdcp3_tiny *q)
{
	struct hvdcp3_limited_info *hli = q->hli;

	if(IS_ERR_OR_NULL(q->usb_psy) || IS_ERR_OR_NULL(q->battery_psy) ||
			IS_ERR_OR_NULL(q->parallel_psy))
		return -EINVAL;

	hli->battery_present = get_int_prop_from_psy(q->battery_psy,
			POWER_SUPPLY_PROP_PRESENT);

	pr_info("get psy usb present\n");
	hli->usb_present = get_int_prop_from_psy(q->usb_psy,
			POWER_SUPPLY_PROP_PRESENT);

	hli->is_usb_hvdcp = is_hvdcp_type(q);

	pr_info("get parallel psy present\n");
	hli->parallel_present = get_int_prop_from_psy(q->parallel_psy,
			POWER_SUPPLY_PROP_PRESENT);

	pr_info("get battery psy input limited\n");
	hli->input_current_limited = get_int_prop_from_psy(q->battery_psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED);
	pr_info("get parralel psy input limited\n");
	hli->parralel_current_limited = get_int_prop_from_psy(q->parallel_psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED);

	hli->usb_uv = get_int_prop_from_psy(q->usb_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW);
	hli->hvdcp_ov = is_hvdcp_ov(hli->usb_uv);

	hli->battery_uv = get_int_prop_from_psy(q->battery_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW);

	hli->max_pulse_allowed = get_int_prop_from_psy(q->battery_psy,
			POWER_SUPPLY_PROP_MAX_PULSE_ALLOWED);

	hli->pulse_cnt = get_int_prop_from_psy(q->battery_psy,
			POWER_SUPPLY_PROP_DP_DM);

#if AUTO_ALGO_OPTI
	hli->icl_down_cnt = 0;
#endif

	hli->aicl_now = get_int_prop_from_psy(q->battery_psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_MAX);

	pr_info("hvdcp %d,usb_uv %d, pulse_cnt %d\n",
			hli->is_usb_hvdcp,hli->usb_uv,hli->pulse_cnt);
	return 0;
}

static int hvdcp3_set_hts(struct hvdcp3_tiny *q,enum hvdcp3_tiny_st st)
{
	if(IS_ERR_OR_NULL(q))
		return -EINVAL;

	if(st < QC3_AUTH_NO_HVDCP3 || st >= QC3_ST_MAX)
		return -EINVAL;

	mutex_lock(&q->ls_lock);
	q->hts = st;
	mutex_unlock(&q->ls_lock);

	return 0;
}
static enum hvdcp3_tiny_st hvdcp3_get_hts(struct hvdcp3_tiny *q)
{
	if(IS_ERR_OR_NULL(q))
		return -EINVAL;

	return q->hts;
}

static int hvdcp3_hls_increase_pulse(struct hvdcp3_tiny *qc3,int ps)
{
	if(IS_ERR_OR_NULL(qc3))
		return -EINVAL;

	while(ps--) {
		if(qc3->pulse_cnt >= qc3_max_pulse_allowed)
			break;

		set_int_prop_to_psy(qc3->battery_psy,
			POWER_SUPPLY_PROP_DP_DM,POWER_SUPPLY_DP_DM_DP_PULSE);

		qc3->pulse_cnt ++;
		msleep(100);
	}

	return 0;
}

static int hvdcp3_hls_decrease_pulse(struct hvdcp3_tiny *qc3,int ps)
{
	while(ps--) {
		if(qc3->pulse_cnt > 0)
			break;

		set_int_prop_to_psy(qc3->battery_psy,
			POWER_SUPPLY_PROP_DP_DM,POWER_SUPPLY_DP_DM_DM_PULSE);

		qc3->pulse_cnt --;
		msleep(60);
	}

	return 0;
}

static int qc3_auth_no_hvdcp3(struct hvdcp3_tiny *qc3)
{
	qc3->pulse_cnt = 0;
	return 0;
}

static int qc3_auth_prepare(struct hvdcp3_tiny *qc3)
{
	if(IS_ERR_OR_NULL(qc3->battery_psy))
		return -EINVAL;

	pr_info("start prepare\n");
	return set_int_prop_to_psy(qc3->battery_psy,
		POWER_SUPPLY_PROP_DP_DM,POWER_SUPPLY_DP_DM_PREPARE);
}

static int qc3_auth_dp_pulse(struct hvdcp3_tiny *qc3)
{
	int pulse = qc3->cmd[QC3_AUTH_DP_PULSE].argv;
	unsigned long timeout;

	qc3->pulse_cnt = 0;

	pr_info("start auth %d\n",pulse);
	while(pulse--) {
		timeout = jiffies +
			msecs_to_jiffies(qc3->cmd[QC3_AUTH_DP_PULSE].delay_ms);

		set_int_prop_to_psy(qc3->battery_psy,POWER_SUPPLY_PROP_DP_DM,
				POWER_SUPPLY_DP_DM_DP_PULSE);

		pr_info("confirm pulse cnt %d\n",qc3->pulse_cnt);
		qc3->pulse_cnt ++;

		while(time_before(jiffies,timeout))
			cpu_relax();
	}
	pr_info("end auth\n");
	return 0;
}

static int qc3_auth_confired(struct hvdcp3_tiny *qc3)
{
	union power_supply_propval ret = {0, };

	pr_info("start confirm\n");
	if(IS_ERR_OR_NULL(qc3))
		return -EINVAL;

	if(IS_ERR_OR_NULL(qc3->usb_psy))
		return -EINVAL;

	ret.intval = get_int_prop_from_psy(qc3->usb_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW);

	pr_info("confirm volate now %d (uv)\n",ret.intval);
	if(ret.intval >= qc3_confirmed_voltage_uv) {
		set_int_prop_to_psy(qc3->battery_psy,POWER_SUPPLY_PROP_DP_DM,
			POWER_SUPPLY_DP_DM_CONFIRMED_HVDCP3);
		pr_info("confirm ok\n");
		return 1;
	}
	pr_info("failed confirm\n");
	return 0;
}

static int qc3_auth_opti_initial_vol(struct hvdcp3_tiny *qc3)
{
	/*need caculate run time*/
	qc3->pulse_opti = qc3_max_pulse_allowed;
	return 0;
}

static int qc3_opti_rerun_aicl(struct hvdcp3_tiny *qc3)
{
	unsigned long timeout = jiffies + 2*HZ;
	if(IS_ERR_OR_NULL(qc3))
		return -EINVAL;

	set_int_prop_to_psy(qc3->battery_psy,POWER_SUPPLY_PROP_RERUN_AICL,1);

	while(time_before(jiffies,timeout))
		cpu_relax();

	return 0;
}

static int qc3_opti_not_limited(struct hvdcp3_tiny *qc3)
{
	unsigned long timeout = jiffies +
		msecs_to_jiffies(qc3->cmd[QC3_OPTI_NOT_LIMITED].delay_ms);

	if(IS_ERR_OR_NULL(qc3))
		return -EINVAL;

	while(time_before(jiffies,timeout))
		cpu_relax();

	return 0;
}

static int qc3_opti_dp_pulse(struct hvdcp3_tiny *qc3)
{
	if(IS_ERR_OR_NULL(qc3))
		return -EINVAL;

	if(qc3->pulse_cnt <= qc3_max_pulse_allowed)
		hvdcp3_hls_increase_pulse(qc3,1);

	return 0;
}

static int qc3_opti_limited(struct hvdcp3_tiny *qc3)
{
	if(IS_ERR_OR_NULL(qc3))
		return -EINVAL;

	if(qc3->hli->input_current_limited)
		return 1;

	return 0;
}

static int qc3_opti_dm_pulse(struct hvdcp3_tiny *qc3)
{
	unsigned long timeout = jiffies +
		msecs_to_jiffies(qc3->cmd[QC3_OPTI_DM_PULSE].delay_ms);

	if(IS_ERR_OR_NULL(qc3))
		return -EINVAL;

	hvdcp3_hls_decrease_pulse(qc3,1);

	while(time_before(jiffies,timeout))
		cpu_relax();

	return 0;
}
static int hvdcp3_do_event(struct hvdcp3_tiny *qc3,enum hvdcp3_tiny_st es)
{
	int rc;

	if(IS_ERR_OR_NULL(qc3))
		return -EINVAL;

	rc = qc3->cmd[es].call(qc3);


	return rc;
}


#if 0
static int hvdcp3_hls_auth_confirmed(struct hvdcp3_tiny *q)
{
	int rc;
	union power_supply_propval ret = {0, };

	if(IS_ERR_OR_NULL(q))
		return -EINVAL;

	if(IS_ERR_OR_NULL(q->usb_psy))
		return -EINVAL;

	rc = q->usb_psy->get_property(q->usb_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
	if(rc) {
		pr_err("Couldn't get usb volatge rc=%d\n",rc);
	}

	if(ret.intval >= qc3_confirmed_voltage_uv) {
		set_int_prop_to_psy(q->battery_psy,POWER_SUPPLY_PROP_DP_DM,
				POWER_SUPPLY_DP_DM_CONFIRMED_HVDCP3);
		return 1;
	}

	return 0;
}

static int hvdcp3_hls_rerun_aicl(struct hvdcp3_tiny *qc3)
{
	if(IS_ERR_OR_NULL(qc3))
		return -EINVAL;

	set_int_prop_to_psy(qc3->battery_psy,POWER_SUPPLY_PROP_RERUN_AICL,1);

	msleep(2000);
	return 0;
}

static int hvdcp3_hls_dp_pulse(struct hvdcp3_tiny *q)
{
	if(IS_ERR_OR_NULL(q))
		return -EINVAL;

	hvdcp3_hls_increase_pulse(q,1);

	msleep(60);

	return 0;
}

static int hvdcp3_hls_dm_pulse(struct hvdcp3_tiny *q)
{
	if(IS_ERR_OR_NULL(q))
		return -EINVAL;

	hvdcp3_hls_decrease_pulse(q,1);
	msleep(60);

	return 0;
}
#endif

static int hvdcp3_handle_hls(struct hvdcp3_tiny *q)
{
	enum hvdcp3_tiny_st hts = hvdcp3_get_hts(q);
	enum hvdcp3_tiny_st next_hts = QC3_AUTH_NO_HVDCP3;
	int rc;

	if(IS_ERR_OR_NULL(q))
		return -EINVAL;

	pm_stay_awake(q->dev);
	if(!q->hli->is_usb_hvdcp) {
		hvdcp3_set_hts(q,QC3_AUTH_NO_HVDCP3);
		return 0;
	}

	switch (hts) {
	case	QC3_AUTH_NO_HVDCP3:
		if(q->hli->is_usb_hvdcp) {
			next_hts = QC3_AUTH_PREPARE;
		}
		break;

	case	QC3_AUTH_PREPARE:
		hvdcp3_do_event(q,QC3_AUTH_PREPARE);
		next_hts = QC3_AUTH_DP_PULSE;
		break;

	case 	QC3_AUTH_DP_PULSE:
		rc = hvdcp3_do_event(q,QC3_AUTH_DP_PULSE);
		if(rc)
			next_hts = QC3_AUTH_NO_HVDCP3;
		else
			next_hts = QC3_AUTH_CONFIRED;
		break;

	case 	QC3_AUTH_CONFIRED:
		rc = hvdcp3_do_event(q,QC3_AUTH_CONFIRED);
		if(rc) {
			next_hts = QC3_AUTH_OPTI_INITIAL_VOL;
		} else {
			next_hts = QC3_AUTH_NO_HVDCP3;
		}
		break;

	case 	QC3_AUTH_OPTI_INITIAL_VOL:
		rc = hvdcp3_do_event(q,QC3_AUTH_CONFIRED);
		if(rc)
			next_hts = QC3_AUTH_NO_HVDCP3;
		else
			next_hts = QC3_OPTI_RERUN_AICL;
		break;

	case 	QC3_OPTI_RERUN_AICL:
		hvdcp3_do_event(q,QC3_OPTI_RERUN_AICL);

		if(q->hli->input_current_limited)
			next_hts = QC3_OPTI_LIMITED;
		else
			next_hts = QC3_OPTI_NOT_LIMITED;
		break;

	case 	QC3_OPTI_NOT_LIMITED:

		hvdcp3_do_event(q,QC3_OPTI_NOT_LIMITED);
		if(q->hli->input_current_limited)
			next_hts = QC3_OPTI_LIMITED;
		else
			next_hts = QC3_OPTI_DM_PULSE;
		break;

	case 	QC3_OPTI_DP_PULSE:
		hvdcp3_do_event(q,QC3_OPTI_DP_PULSE);
		if(q->hli->input_current_limited)
			next_hts = QC3_OPTI_DP_PULSE;
		else
			next_hts = QC3_OPTI_RERUN_AICL;
		break;
	case 	QC3_OPTI_LIMITED:

		rc = hvdcp3_do_event(q,QC3_OPTI_LIMITED);

		if(rc)
			next_hts = QC3_OPTI_DP_PULSE;
		else
			next_hts = QC3_OPTI_RERUN_AICL;
		break;
	case 	QC3_OPTI_DM_PULSE:
		hvdcp3_do_event(q,QC3_OPTI_DM_PULSE);
		if(q->hli->input_current_limited)
			next_hts = QC3_OPTI_LIMITED;
		else
			next_hts = QC3_OPTI_RERUN_AICL;
		break;
	default:
		next_hts = QC3_AUTH_NO_HVDCP3;
		break;
	}

	pr_info("ev: %d -> %d\n",hts,next_hts);
	/* hvdcp3_do_event(q,hts); */

	hvdcp3_set_hts(q,next_hts);
	pm_relax(q->dev);
	return 0;
}
static int hvdcp3_tiny_thread(void *data)
{
	struct hvdcp3_tiny *qc3_tiny = data;
	long timeout = -1;

	if(IS_ERR_OR_NULL(qc3_tiny))
		return -EINVAL;

	while (!kthread_should_stop()) {

		if(qc3_tiny->hli->is_usb_hvdcp)
			timeout = msecs_to_jiffies(2000);
		else
			timeout = -1;

		pr_info("hvdcp3 thread running\n");
		wait_event_timeout(qc3_tiny->wait,
				qc3_tiny->usb_insert,timeout);

		qc3_tiny->usb_insert = 0;
		hvdcp3_get_limited_info(qc3_tiny);

		qc3_tiny->handle(qc3_tiny);
	}
	return 0;
}

static int hvdc3_parse_dt(struct device *dev)
{
	struct device_node *node = dev->of_node;
	struct hvdcp3_tiny *qc = dev_get_drvdata(dev);

	if(IS_ERR_OR_NULL(node))
		return -EINVAL;

	qc->hvdcp3_allowed = of_property_read_bool(node,"qc3,opti-voltage");

	return 0;
}

static int charger_notifier_call(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	struct hvdcp3_tiny *qc3_tiny = container_of(nb, struct hvdcp3_tiny , nb);
	struct power_supply *psy = data;

	if(IS_ERR_OR_NULL(qc3_tiny) || IS_ERR_OR_NULL(psy))
		return -EINVAL;

	if (0 == strcmp(psy->name,"usb"))
	{
		qc3_tiny->usb_insert = 1;
		wake_up(&qc3_tiny->wait);
	}

	return NOTIFY_OK;
}

static int hvdcp3_tiny_probe(struct platform_device *pdev)
{
	struct hvdcp3_limited_info *limit_info;
	struct hvdcp3_tiny	*qc3_tiny;
	int rc;

	pr_info("start\n");
	limit_info = devm_kzalloc(&pdev->dev, sizeof(*limit_info), GFP_KERNEL);

	qc3_tiny = devm_kzalloc(&pdev->dev, sizeof(*qc3_tiny), GFP_KERNEL);

	qc3_tiny->dev = &pdev->dev;
	qc3_tiny->usb_psy = power_supply_get_by_name("usb");
	qc3_tiny->battery_psy = power_supply_get_by_name("battery");
	qc3_tiny->parallel_psy = power_supply_get_by_name("usb-parallel");

	if(IS_ERR_OR_NULL(qc3_tiny->usb_psy) ||
		IS_ERR_OR_NULL(qc3_tiny->battery_psy) ||
		IS_ERR_OR_NULL(qc3_tiny->parallel_psy)) {
		pr_err("phy failed,probe defer\n");
		return -EPROBE_DEFER;
	}

	platform_set_drvdata(pdev, qc3_tiny);

	hvdc3_parse_dt(qc3_tiny->dev);

	init_waitqueue_head(&qc3_tiny->wait);

	/* max vbus(9v) = 5v + 20*200mv */
	qc3_tiny->pulse_cnt = 0;
	qc3_tiny->usb_insert = 0;
	mutex_init(&qc3_tiny->ls_lock);
	device_init_wakeup(qc3_tiny->dev, true);

	qc3_tiny->handle = hvdcp3_handle_hls;
	qc3_tiny->cmd = default_vendor_cmd;

	qc3_tiny->hts = QC3_AUTH_NO_HVDCP3;
	qc3_tiny->hli = limit_info;

	qc3_tiny->task = kthread_create(hvdcp3_tiny_thread, qc3_tiny, "%s",
			dev_name(qc3_tiny->dev));

	if(IS_ERR_OR_NULL(qc3_tiny->task)) {
		pr_err("Can't create qc3 tiny thread\n");
		return -1;
	}

	qc3_tiny->nb.notifier_call = charger_notifier_call;
	rc = power_supply_reg_notifier(&qc3_tiny->nb);
	if (rc < 0) {
		pr_err("Couldn't register psy notifier rc=%d\n",rc);
	}

	wake_up_process(qc3_tiny->task);
	pr_info("probe ok\n");
	return 0;
}

static int hvdcp3_tiny_remove(struct platform_device *pdev)
{
	return 0;
}
static struct platform_driver hvdcp3_tiny_driver = {
	.driver = {
		.name = "hvdcp3-tiny",
		.owner = THIS_MODULE,
		.pm = &hvdcp3_tiny_pm,
		.of_match_table = hvdcp3_tiny_match,
	},
	.probe = hvdcp3_tiny_probe,
	.remove = hvdcp3_tiny_remove,
	.id_table = hvdcp3_tiny_id,
};



static int __init hvdcp3_tiny_init(void)
{
	return platform_driver_register(&hvdcp3_tiny_driver);
}

late_initcall(hvdcp3_tiny_init);
