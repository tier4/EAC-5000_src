/*
 * max96712.c - max96712 IO Expander driver
 *
 * Copyright (c) 2016-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/* #define DEBUG */

#ifndef __SL_DESERIALIZER__
#define __SL_DESERIALIZER__
#define __MAX96712__
#else
#error Error, a deserializer is already compiled. Fix the defconfig and use only one deserializer.
#endif

#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <media/camera_common.h>
#include <linux/module.h>
#include "max96712_mode_tbls.h"
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

struct max96712
{
    struct i2c_client *i2c_client;
    struct regmap *regmap;
    u32 max_address; // MAX deser address from dts (required to update addr in tables)
    int channel; // channel id from dts
    struct dentry *dp;
    struct dentry *fp;
};
struct max96712 *global_priv[4];

int write_reg_Dser(int slaveAddr, int channel, u16 addr, u8 val)
{
    struct i2c_client *i2c_client = NULL;
    int bak = 0;
    int err;
    /* unsigned int ival = 0; */

    if (channel > 3 || channel < 0 || global_priv[channel] == NULL)
        return -1;
    i2c_client = global_priv[channel]->i2c_client;
    bak = i2c_client->addr;

    i2c_client->addr = slaveAddr;
    err = regmap_write(global_priv[channel]->regmap, addr, val);

    i2c_client->addr = bak;
    if (err)
    {
        dev_err(&i2c_client->dev, "%s: addr = 0x%x, val = 0x%x\n",
                __func__, addr, val);
        return -1;
    }
    return 0;
}
EXPORT_SYMBOL(write_reg_Dser);

int read_reg_Dser(int slaveAddr, int channel, u16 addr, unsigned int *val)
{
    struct i2c_client *i2c_client = NULL;
    int bak = 0;
    int err;
    if (channel > 3 || channel < 0 || global_priv[channel] == NULL)
        return -1;
    i2c_client = global_priv[channel]->i2c_client;
    bak = i2c_client->addr;
    i2c_client->addr = slaveAddr;
    err = regmap_read(global_priv[channel]->regmap, addr, val);
    i2c_client->addr = bak;
    if (err)
    {
        dev_err(&i2c_client->dev, "%s: addr = 0x%x, val = 0x%x\n",
                __func__, addr, *val);
        return -1;
    }
    return 0;
}
EXPORT_SYMBOL(read_reg_Dser);


int links_check_Dser(int channel, u8 *links)
{
    unsigned int link,ret;
    int tab_id = MAX96712_LINK_REGS;
    int err;
    u8 i,j;

    if (links == NULL)
        return -1;

    if (channel > 3 || channel < 0 || global_priv[channel] == NULL)
        return -1;

    // Initialize the indice and the return value
    j = 0;
    ret = 0;

    for (i=0; i<4; i++)
    {
	    err = read_reg_Dser(global_priv[channel]->max_address, channel, mode_table[tab_id][i].addr, &link);

        if (err)
            return -1;

        // Bit mask to get the essential information: is link i connected?
        link = (link & 0x08) >> 3;
        // increments the number of connected links accordingly
        ret += link;

	    if (link)
        {
            links[j] = i;
            j++;
        }
    }
    return ret;
}
EXPORT_SYMBOL(links_check_Dser);

int links_dis_Dser(int channel)
{
    int err;

    if (channel > 3 || channel < 0 || global_priv[channel] == NULL)
        return -1;

    err = write_reg_Dser(global_priv[channel]->max_address, channel, GMSL_LINKS_EN_REG, 0xF0);

    if (err)
        return -1;

    msleep(300);

    return 0;
}
EXPORT_SYMBOL(links_dis_Dser);

int cc_en_Dser(int channel, u8 link)
{
    int err;
    u8 val;

    if (channel > 3 || channel < 0 || global_priv[channel] == NULL)
        return -1;

    val = 0xFF ^ (1<<(2*link));

    err = write_reg_Dser(global_priv[channel]->max_address, channel, GMSL_LINKS_CC_REG, val);

    if (err)
        return -1;

    msleep(100);

    return 0;
}
EXPORT_SYMBOL(cc_en_Dser);

int i2c_setup_Dser(int channel, u8* links)
{
    int err;
    u8 val = 0xFF;

    if (channel > 3 || channel < 0 || global_priv[channel] == NULL)
        return -1;

    err = write_reg_Dser(global_priv[channel]->max_address, channel, GMSL_LINKS_CC_REG, val);

    val = 1<<(4+links[1]);

    err = write_reg_Dser(global_priv[channel]->max_address, channel, GMSL_CC_X_OVR_REG, val);

    val = 0xFF ^ (1<<(2*links[0]));

    val =  val ^ (2<<(2*links[1]));

    err = write_reg_Dser(global_priv[channel]->max_address, channel, GMSL_LINKS_CC_REG, val);

    if (err)
        return -1;

    msleep(100);

    return 0;
}
EXPORT_SYMBOL(i2c_setup_Dser);

int pipes_01_en_Dser(int channel, u8 link)
{
    int err;
    unsigned int ival;
    u8 val;

    if (channel > 3 || channel < 0 || global_priv[channel] == NULL)
        return -1;

    err = read_reg_Dser(global_priv[channel]->max_address, channel, GMSL_LINKS_EN_REG, &ival);

    if (err)
        return -1;

    val = 1<<link;

    val = (u8) ival | val;

    err = write_reg_Dser(global_priv[channel]->max_address, channel, GMSL_LINKS_EN_REG, val);

    if (err)
        return -1;

    msleep(300);

    val = PIPES_XZ_MASK | (link<<2) | (link<<6);

    err = write_reg_Dser(global_priv[channel]->max_address, channel, GMSL_PIPES_01_REG, val);

    if (err)
        return -1;

    msleep(100);

    return 0;
}
EXPORT_SYMBOL(pipes_01_en_Dser);

int pipes_23_en_Dser(int channel, u8 link)
{
    int err;
    unsigned int ival;
    u8 val;

    if (channel > 3 || channel < 0 || global_priv[channel] == NULL)
        return -1;

    err = read_reg_Dser(global_priv[channel]->max_address, channel, GMSL_LINKS_EN_REG, &ival);

    if (err)
        return -1;

    val = 1<<link;

    val = (u8) ival | val;

    err = write_reg_Dser(global_priv[channel]->max_address, channel, GMSL_LINKS_EN_REG, val);

    if (err)
        return -1;

    msleep(300);
    val = PIPES_XZ_MASK | (link<<2) | (link<<6);

    err = write_reg_Dser(global_priv[channel]->max_address, channel, GMSL_PIPES_23_REG, val);

    if (err)
        return -1;

    msleep(100);

    return 0;
}
EXPORT_SYMBOL(pipes_23_en_Dser);

static int max96712_read_reg(struct max96712 *priv, u16 addr,
        unsigned int *val)
{
    struct i2c_client *i2c_client = priv->i2c_client;
    int err;

    err = regmap_read(priv->regmap, addr, val);
    if (err)
        dev_err(&i2c_client->dev, "%s:i2c read failed, 0x%x = %x\n",
                __func__, addr, *val);

    return err;
}

static int max96712_write_table(struct max96712 *priv,
        const struct index_reg_8 table[])
{
    struct i2c_client *i2c_client = priv->i2c_client;
    int i = 0, j = 0;
    int ret = 0;
    int retry = 5;

    // While we haven't reach the end of the table
    while (table[i].addr != MAX96712_TABLE_END)
    {
        // While we haven't tried to write the register 'retry' times
        for (j = 0; j < retry; j++)
        {
            // We try to write the register. 0 means i2c master
            ret = write_reg_Dser(priv->max_address, priv->channel, table[i].addr, (u8)table[i].val);
            // if the return value is bad
            if (ret && (table[i].addr != 0x0000))
            {
                dev_warn(&i2c_client->dev, "write_reg_Dser: try %d\n", j);
                msleep(4);
                // if we have retried 'retry' number of time we exit
                if (j == retry-1)
                    return -1;
                //else, we retry
                continue;
            }
            // If we write a reset register
            if (0x0013 == table[i].addr || 0x0000 == table[i].addr ||
                    0x0006 == table[i].addr || 0x0018 == table[i].addr)
                msleep(300);
            // If it is just a regular register
            else
                msleep(100);
            // Everything went well so we don't have to retry
            break;
        }
        i++;
    }
    return 0;
}

int fast_reset_Dser(int channel)
{
    int err;

    if (channel > 3 || channel < 0 || global_priv[channel] == NULL)
        return -1;

    err = max96712_write_table(global_priv[channel], mode_table[MAX96712_FST_RST]);

    if (err)
        return -1;

    return 0;
}
EXPORT_SYMBOL(fast_reset_Dser);

int slow_reset_Dser(int channel)
{
    int err;

    if (channel > 3 || channel < 0 || global_priv[channel] == NULL)
        return -1;

    err = max96712_write_table(global_priv[channel], mode_table[MAX96712_INIT]);

    if (err)
        return -1;

    return 0;
}
EXPORT_SYMBOL(slow_reset_Dser);

u32 fps_set_Dser(int channel, s64 val)
{
    int err = -1;
    int tab_id = -1;
    u32 ret = 0xFFFF;

    if (channel > 3 || channel < 0 || global_priv[channel] == NULL)
        return -1;

    // 30fps full resolution
    if (val == 30000000)
    {
        tab_id = MAX96712_30_FPS;
                ret = 0x61A;
    }

    // 60fps full resolution
    if (val == 60000000)
    {
        tab_id = MAX96712_60_FPS;
                ret = 0x30D;
    }

    // 120fps full resolution
    if (val == 120000000)
    {
        tab_id = MAX96712_120_FPS;
                ret = 0x186;
    }

    // It is not really an error to ask for a wrong fps value
    if (tab_id == -1)
    {
        dev_warn(&global_priv[channel]->i2c_client->dev,
                "%s: %lld is not a supported value. [30,60,120]*10^6 are supported.\n",
                __func__, val);
        return 0xEEEE;
    }

    err = max96712_write_table(global_priv[channel], mode_table[tab_id]);

    if (err)
        return 0xFFFF;

    return ret;
}
EXPORT_SYMBOL(fps_set_Dser);

static int max96712_stats_show(struct seq_file *s, void *data)
{
    return 0;
}

static int max96712_debugfs_open(struct inode *inode, struct file *file)
{
    return single_open(file, max96712_stats_show, inode->i_private);
}

static ssize_t max96712_debugfs_write(struct file *s,
        const char __user *user_buf,
        size_t count, loff_t *ppos)
{
    struct max96712 *priv =
        ((struct seq_file *)s->private_data)->private;
    struct i2c_client *i2c_client = priv->i2c_client;

    char buf[255];
    int buf_size;
    int val = 0;

    if (!user_buf || count <= 1)
        return -EFAULT;

    memset(buf, 0, sizeof(buf));
    buf_size = min(count, sizeof(buf) - 1);
    if (copy_from_user(buf, user_buf, buf_size))
        return -EFAULT;

    if (buf[0] == 'd')
    {
        dev_info(&i2c_client->dev, "%s, set daymode\n", __func__);
        max96712_read_reg(priv, 0x0010, &val);
        return count;
    }

    if (buf[0] == 'n')
    {
        dev_info(&i2c_client->dev, "%s, set nightmode\n", __func__);
        return count;
    }

    return count;
}

static const struct file_operations max96712_debugfs_fops = {
    .open = max96712_debugfs_open,
    .read = seq_read,
    .write = max96712_debugfs_write,
    .llseek = seq_lseek,
    .release = single_release,
};

static int max96712_debugfs_init(const char *dir_name,
        struct dentry **d_entry,
        struct dentry **f_entry,
        struct max96712 *priv)
{
    struct dentry *dp, *fp;
    char dev_name[20];
    struct i2c_client *i2c_client = priv->i2c_client;
    struct device_node *np = i2c_client->dev.of_node;
    int err = 0;
    const char *str;

    if (np)
    {
        err = of_property_read_string(np, "channel", &str);
        if (err)
            dev_err(&i2c_client->dev, "channel not found --> Requires 'channel' entry in DTS\n");
        err = of_property_read_u32(np, "reg", &priv->max_address);
        if (err)
            dev_err(&i2c_client->dev, "MAX96712 address not found --> Requires 'reg' entry in DTS %d\n",err);

        err = snprintf(dev_name, sizeof(dev_name), "max96712_%s",
                str);
        if (err < 0)
            return -EINVAL;
    }
    priv->channel = str[0] - 'a';
    if (priv->channel < 0 ||  priv->channel > 3 )
        return -EINVAL;
    global_priv[priv->channel] = priv;

    //dev_dbg(&i2c_client->dev, "%s: index %d\n", __func__, priv->channel);

    dp = debugfs_create_dir(dev_name, NULL);
    if (dp == NULL)
    {
        dev_err(&i2c_client->dev, "%s: debugfs create dir failed\n",
                __func__);
        return -ENOMEM;
    }

    fp = debugfs_create_file("max96712", S_IRUGO | S_IWUSR,
            dp, priv, &max96712_debugfs_fops);
    if (!fp)
    {
        dev_err(&i2c_client->dev, "%s: debugfs create file failed\n",
                __func__);
        debugfs_remove_recursive(dp);
        return -ENOMEM;
    }

    if (d_entry)
        *d_entry = dp;
    if (f_entry)
        *f_entry = fp;
    return 0;
}

static struct regmap_config max96712_regmap_config = {
    .reg_bits = 16,
    .val_bits = 8,
    .cache_type = REGCACHE_NONE, //No cache for proper reset
};

static int max96712_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    struct max96712 *priv;
    int err = 0;

    dev_info(&client->dev, "%s: enter\n", __func__);

    priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
    priv->i2c_client = client;
    priv->regmap = devm_regmap_init_i2c(priv->i2c_client,
            &max96712_regmap_config);
    if (IS_ERR(priv->regmap))
    {
        dev_err(&client->dev,
                "regmap init failed: %ld\n", PTR_ERR(priv->regmap));
        return -ENODEV;
    }

    err = max96712_debugfs_init(NULL, &priv->dp, &priv->fp, priv);
    if (err)
        return err;

    /*set daymode by fault*/
    dev_info(&client->dev, "%s:  success\n", __func__);
    return err;
}

    static int
max96712_remove(struct i2c_client *client)
{
    int i;
    for (i = 0; i < 4; i++)
    {
        if (global_priv[i] != NULL)
        {
            dev_info(&client->dev, "%s: removing debugfs for max96712 %i\n", __func__ ,i);
            debugfs_remove_recursive(global_priv[i]->fp);
            debugfs_remove_recursive(global_priv[i]->dp);
        }
    }

    dev_info(&client->dev, "%s: success\n", __func__);

    //
    //  Everything is automatically deallocated.
    //

    return 0;
}

static const struct i2c_device_id max96712_id[] = {
    {"max96712", 0},
    {},
};

const struct of_device_id max96712_of_match[] = {
    {
        .compatible = "stereolabs,max96712",
    },
    {},
};

MODULE_DEVICE_TABLE(i2c, max96712_id);

static struct i2c_driver max96712_i2c_driver = {
    .driver = {
        .name = "max96712",
        .owner = THIS_MODULE,
    },
    .probe = max96712_probe,
    .remove = max96712_remove,
    .id_table = max96712_id,
};

static int __init max96712_init(void)
{
    return i2c_add_driver(&max96712_i2c_driver);
}

static void __exit max96712_exit(void)
{
    i2c_del_driver(&max96712_i2c_driver);
}

module_init(max96712_init);
module_exit(max96712_exit);

MODULE_DESCRIPTION("IO Expander driver max96712");
MODULE_AUTHOR("Stereolabs");
MODULE_LICENSE("GPL v2");
