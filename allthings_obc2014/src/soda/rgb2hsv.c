
#include "rgb2hsv.h"

RgbColor HsvToRgb(HsvColor hsv)
{
    RgbColor rgb;
    unsigned char region, remainder, p, q, t;

    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6; 

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            rgb.r = hsv.v; rgb.g = t; rgb.b = p;
            break;
        case 1:
            rgb.r = q; rgb.g = hsv.v; rgb.b = p;
            break;
        case 2:
            rgb.r = p; rgb.g = hsv.v; rgb.b = t;
            break;
        case 3:
            rgb.r = p; rgb.g = q; rgb.b = hsv.v;
            break;
        case 4:
            rgb.r = t; rgb.g = p; rgb.b = hsv.v;
            break;
        default:
            rgb.r = hsv.v; rgb.g = p; rgb.b = q;
            break;
    }

    return rgb;
}

HsvColor RgbToHsv(RgbColor rgb)
{
    HsvColor hsv;
    unsigned char rgbMin, rgbMax;

    rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
    rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

    hsv.v = rgbMax;
    if (hsv.v == 0)
    {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * long(rgbMax - rgbMin) / hsv.v;
    if (hsv.s == 0)
    {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == rgb.r)
        hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
    else if (rgbMax == rgb.g)
        hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

    return hsv;
}


void RgbToHsvP(unsigned char* src, unsigned char* dst)
{
    unsigned char rgbMin, rgbMax;

    rgbMin = src[0] < src[1] ? (src[0] < src[2] ? src[0] : src[2]) : (src[1] < src[2] ? src[1] : src[2]);
    rgbMax = src[0] > src[1] ? (src[0] > src[2] ? src[0] : src[2]) : (src[1] > src[2] ? src[1] : src[2]);

    dst[2] = rgbMax;
    if (dst[2] == 0)
    {
        dst[0] = 0;
        dst[1] = 0;
        return;
    }

    dst[1] = 255 * long(rgbMax - rgbMin) / dst[2];
    if (dst[1] == 0)
    {
        dst[0] = 0;
        return;
    }

    if (rgbMax == src[0])
        dst[0] = 0 + 43 * (src[1] - src[2]) / (rgbMax - rgbMin);
    else if (rgbMax == src[1])
        dst[0] = 85 + 43 * (src[2] - src[0]) / (rgbMax - rgbMin);
    else
        dst[0] = 171 + 43 * (src[0] - src[1]) / (rgbMax - rgbMin);

}

