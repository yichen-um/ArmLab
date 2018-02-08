// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "dynamixel_config_list_t.h"

static int __dynamixel_config_list_t_hash_computed;
static int64_t __dynamixel_config_list_t_hash;

int64_t __dynamixel_config_list_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __dynamixel_config_list_t_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__dynamixel_config_list_t_get_hash;
    (void) cp;

    int64_t hash = (int64_t)0xf7de54a48e3094eaLL
         + __int32_t_hash_recursive(&cp)
         + __dynamixel_config_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __dynamixel_config_list_t_get_hash(void)
{
    if (!__dynamixel_config_list_t_hash_computed) {
        __dynamixel_config_list_t_hash = __dynamixel_config_list_t_hash_recursive(NULL);
        __dynamixel_config_list_t_hash_computed = 1;
    }

    return __dynamixel_config_list_t_hash;
}

int __dynamixel_config_list_t_encode_array(void *buf, int offset, int maxlen, const dynamixel_config_list_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].len), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __dynamixel_config_t_encode_array(buf, offset + pos, maxlen - pos, p[element].configs, p[element].len);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int dynamixel_config_list_t_encode(void *buf, int offset, int maxlen, const dynamixel_config_list_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __dynamixel_config_list_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __dynamixel_config_list_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __dynamixel_config_list_t_encoded_array_size(const dynamixel_config_list_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __int32_t_encoded_array_size(&(p[element].len), 1);

        size += __dynamixel_config_t_encoded_array_size(p[element].configs, p[element].len);

    }
    return size;
}

int dynamixel_config_list_t_encoded_size(const dynamixel_config_list_t *p)
{
    return 8 + __dynamixel_config_list_t_encoded_array_size(p, 1);
}

size_t dynamixel_config_list_t_struct_size(void)
{
    return sizeof(dynamixel_config_list_t);
}

int dynamixel_config_list_t_num_fields(void)
{
    return 2;
}

int dynamixel_config_list_t_get_field(const dynamixel_config_list_t *p, int i, lcm_field_t *f)
{
    if (0 > i || i >= dynamixel_config_list_t_num_fields())
        return 1;
    
    switch (i) {
    
        case 0: {
            f->name = "len";
            f->type = LCM_FIELD_INT32_T;
            f->typestr = "int32_t";
            f->num_dim = 0;
            f->data = (void *) &p->len;
            return 0;
        }
        
        case 1: {
            /* dynamixel_config_t */
            f->name = "configs";
            f->type = LCM_FIELD_USER_TYPE;
            f->typestr = "dynamixel_config_t";
            f->num_dim = 1;
            f->dim_size[0] = p->len;
            f->dim_is_variable[0] = 1;
            f->data = (void *) &p->configs;
            return 0;
        }
        
        default:
            return 1;
    }
}

const lcm_type_info_t *dynamixel_config_list_t_get_type_info(void)
{
    static int init = 0;
    static lcm_type_info_t typeinfo;
    if (!init) {
        typeinfo.encode         = (lcm_encode_t) dynamixel_config_list_t_encode;
        typeinfo.decode         = (lcm_decode_t) dynamixel_config_list_t_decode;
        typeinfo.decode_cleanup = (lcm_decode_cleanup_t) dynamixel_config_list_t_decode_cleanup;
        typeinfo.encoded_size   = (lcm_encoded_size_t) dynamixel_config_list_t_encoded_size;
        typeinfo.struct_size    = (lcm_struct_size_t)  dynamixel_config_list_t_struct_size;
        typeinfo.num_fields     = (lcm_num_fields_t) dynamixel_config_list_t_num_fields;
        typeinfo.get_field      = (lcm_get_field_t) dynamixel_config_list_t_get_field;
        typeinfo.get_hash       = (lcm_get_hash_t) __dynamixel_config_list_t_get_hash;
    }
    
    return &typeinfo;
}
int __dynamixel_config_list_t_decode_array(const void *buf, int offset, int maxlen, dynamixel_config_list_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].len), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        p[element].configs = (dynamixel_config_t*) lcm_malloc(sizeof(dynamixel_config_t) * p[element].len);
        thislen = __dynamixel_config_t_decode_array(buf, offset + pos, maxlen - pos, p[element].configs, p[element].len);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __dynamixel_config_list_t_decode_array_cleanup(dynamixel_config_list_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int32_t_decode_array_cleanup(&(p[element].len), 1);

        __dynamixel_config_t_decode_array_cleanup(p[element].configs, p[element].len);
        if (p[element].configs) free(p[element].configs);

    }
    return 0;
}

int dynamixel_config_list_t_decode(const void *buf, int offset, int maxlen, dynamixel_config_list_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __dynamixel_config_list_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __dynamixel_config_list_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int dynamixel_config_list_t_decode_cleanup(dynamixel_config_list_t *p)
{
    return __dynamixel_config_list_t_decode_array_cleanup(p, 1);
}

int __dynamixel_config_list_t_clone_array(const dynamixel_config_list_t *p, dynamixel_config_list_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int32_t_clone_array(&(p[element].len), &(q[element].len), 1);

        q[element].configs = (dynamixel_config_t*) lcm_malloc(sizeof(dynamixel_config_t) * q[element].len);
        __dynamixel_config_t_clone_array(p[element].configs, q[element].configs, p[element].len);

    }
    return 0;
}

dynamixel_config_list_t *dynamixel_config_list_t_copy(const dynamixel_config_list_t *p)
{
    dynamixel_config_list_t *q = (dynamixel_config_list_t*) malloc(sizeof(dynamixel_config_list_t));
    __dynamixel_config_list_t_clone_array(p, q, 1);
    return q;
}

void dynamixel_config_list_t_destroy(dynamixel_config_list_t *p)
{
    __dynamixel_config_list_t_decode_array_cleanup(p, 1);
    free(p);
}

int dynamixel_config_list_t_publish(lcm_t *lc, const char *channel, const dynamixel_config_list_t *p)
{
      int max_data_size = dynamixel_config_list_t_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = dynamixel_config_list_t_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _dynamixel_config_list_t_subscription_t {
    dynamixel_config_list_t_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void dynamixel_config_list_t_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    dynamixel_config_list_t p;
    memset(&p, 0, sizeof(dynamixel_config_list_t));
    status = dynamixel_config_list_t_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding dynamixel_config_list_t!!!\n", status);
        return;
    }

    dynamixel_config_list_t_subscription_t *h = (dynamixel_config_list_t_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    dynamixel_config_list_t_decode_cleanup (&p);
}

dynamixel_config_list_t_subscription_t* dynamixel_config_list_t_subscribe (lcm_t *lcm,
                    const char *channel,
                    dynamixel_config_list_t_handler_t f, void *userdata)
{
    dynamixel_config_list_t_subscription_t *n = (dynamixel_config_list_t_subscription_t*)
                       malloc(sizeof(dynamixel_config_list_t_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 dynamixel_config_list_t_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg dynamixel_config_list_t LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int dynamixel_config_list_t_subscription_set_queue_capacity (dynamixel_config_list_t_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int dynamixel_config_list_t_unsubscribe(lcm_t *lcm, dynamixel_config_list_t_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe dynamixel_config_list_t_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}
