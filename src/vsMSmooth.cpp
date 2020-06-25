#include <memory>

#include "avisynth.h"
#include "avs/minmax.h"

class vsMSmooth : public GenericVideoFilter
{
	double _threshold;
	double _strength;
	bool _mask;
	bool _luma, _chroma;
	bool processPlane[3];
	bool has_at_least_v8;

    template <typename T, bool rgb>
    void edgeMask(PVideoFrame& mask, PVideoFrame& src, IScriptEnvironment* env);
    template <typename T>
    void smooth(PVideoFrame& dst, PVideoFrame& src, PVideoFrame& mask, IScriptEnvironment* env);

public:
    vsMSmooth(PClip _child, double threshold, double strength, bool mask, bool luma, bool chroma, IScriptEnvironment* env);
	PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env);
	int __stdcall SetCacheHints(int cachehints, int frame_range)
	{
		return cachehints == CACHE_GET_MTMODE ? MT_MULTI_INSTANCE : 0;
	}
};

template <typename T>
static inline void blur3x3(void* maskpv, const void* srcpv, int mask_stride, int stride, int width, int height)
{
    T* maskp = (T*)maskpv;
    const T* srcp = (const T*)srcpv;

    stride /= sizeof(T);
    mask_stride /= sizeof(T);

    maskp[0] = (srcp[0] + srcp[1] +
        srcp[stride] + srcp[stride + 1]) / 4;

    for (int x = 1; x < width - 1; x++)
        maskp[x] = (srcp[x - 1] + srcp[x] + srcp[x + 1] +
            srcp[x + stride - 1] + srcp[x + stride] + srcp[x + stride + 1]) / 6;

    maskp[width - 1] = (srcp[width - 2] + srcp[width - 1] +
        srcp[width + stride - 2] + srcp[width + stride - 1]) / 4;

    srcp += stride;
    maskp += mask_stride;

    for (int y = 1; y < height - 1; y++) {
        maskp[0] = (srcp[-stride] + srcp[-stride + 1] +
            srcp[0] + srcp[1] +
            srcp[stride] + srcp[stride + 1]) / 6;

        for (int x = 1; x < width - 1; x++)
            maskp[x] = (srcp[x - stride - 1] + srcp[x - stride] + srcp[x - stride + 1] +
                srcp[x - 1] + srcp[x] + srcp[x + 1] +
                srcp[x + stride - 1] + srcp[x + stride] + srcp[x + stride + 1]) / 9;

        maskp[width - 1] = (srcp[width - stride - 2] + srcp[width - stride - 1] +
            srcp[width - 2] + srcp[width - 1] +
            srcp[width + stride - 2] + srcp[width + stride - 1]) / 6;

        srcp += stride;
        maskp += mask_stride;
    }

    maskp[0] = (srcp[-stride] + srcp[-stride + 1] +
        srcp[0] + srcp[1]) / 4;

    for (int x = 1; x < width - 1; x++)
        maskp[x] = (srcp[x - stride - 1] + srcp[x - stride] + srcp[x - stride + 1] +
            srcp[x - 1] + srcp[x] + srcp[x + 1]) / 6;

    maskp[width - 1] = (srcp[width - stride - 2] + srcp[width - stride - 1] +
        srcp[width - 2] + srcp[width - 1]) / 4;
}

template <typename T, bool rgb>
static inline void findEdges(void** maskpv, int stride, int width, int height, int th, int maximum)
{
    T* maskp[3];
    maskp[0] = (T*)maskpv[0];
    maskp[1] = (T*)maskpv[1];
    maskp[2] = (T*)maskpv[2];

    stride /= sizeof(T);

    for (int y = 0; y < height - 1; y++) {
        for (int x = 0; x < width - 1; x++) {
            int edge = abs(maskp[0][x] - maskp[0][x + stride + 1]) >= th ||
                abs(maskp[0][x + 1] - maskp[0][x + stride]) >= th ||
                abs(maskp[0][x] - maskp[0][x + 1]) >= th ||
                abs(maskp[0][x] - maskp[0][x + stride]) >= th;
            if (rgb) {
                int edge2 = abs(maskp[1][x] - maskp[1][x + stride + 1]) >= th ||
                    abs(maskp[1][x + 1] - maskp[1][x + stride]) >= th ||
                    abs(maskp[1][x] - maskp[1][x + 1]) >= th ||
                    abs(maskp[1][x] - maskp[1][x + stride]) >= th;
                int edge3 = abs(maskp[2][x] - maskp[2][x + stride + 1]) >= th ||
                    abs(maskp[2][x + 1] - maskp[2][x + stride]) >= th ||
                    abs(maskp[2][x] - maskp[2][x + 1]) >= th ||
                    abs(maskp[2][x] - maskp[2][x + stride]) >= th;
                edge = edge || edge2 || edge3;
            }

            if (edge)
                maskp[0][x] = maximum;
            else
                maskp[0][x] = 0;
        }

        int edge = abs(maskp[0][width - 1] - maskp[0][width + stride - 1]) >= th;
        if (rgb) {
            int edge2 = abs(maskp[1][width - 1] - maskp[1][width + stride - 1]) >= th;
            int edge3 = abs(maskp[2][width - 1] - maskp[2][width + stride - 1]) >= th;
            edge = edge || edge2 || edge3;
        }

        if (edge)
            maskp[0][width - 1] = maximum;
        else
            maskp[0][width - 1] = 0;

        maskp[0] += stride;
        if (rgb) {
            maskp[1] += stride;
            maskp[2] += stride;
        }
    }

    for (int x = 0; x < width - 1; x++) {
        int edge = abs(maskp[0][x] - maskp[0][x + 1]) >= th;
        if (rgb) {
            int edge2 = abs(maskp[1][x] - maskp[1][x + 1]) >= th;
            int edge3 = abs(maskp[2][x] - maskp[2][x + 1]) >= th;
            edge = edge || edge2 || edge3;
        }

        if (edge)
            maskp[0][x] = maximum;
        else
            maskp[0][x] = 0;
    }

    maskp[0][width - 1] = maximum;
}

template <typename T>
static inline void copyMask(void* dstpv, const void* srcpv, int dst_stride, int src_stride, int width, int height, int subSamplingW, int subSamplingH)
{
    T* dstp = (T*)dstpv;
    const T* srcp = (const T*)srcpv;

    src_stride /= sizeof(T);
    dst_stride /= sizeof(T);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++)
            dstp[x] = srcp[x << subSamplingW];

        dstp += dst_stride;
        srcp += static_cast<int64_t>(src_stride) << subSamplingH;
    }
}

static inline int clamp(int val, int minimum, int maximum)
{
    if (val < minimum)
        val = minimum;
    else if (val > maximum)
        val = maximum;

    return val;
}

template <typename T, bool rgb>
void vsMSmooth::edgeMask(PVideoFrame& mask, PVideoFrame& src, IScriptEnvironment* env)
{
    const uint8_t* srcp[3];
    uint8_t* maskp[3];

    int plane = vi.IsRGB() ? 64 : 1;

    const int stride = src->GetPitch(plane);
    const int mask_stride = mask->GetPitch(plane);
    const int width = src->GetRowSize(plane) / vi.ComponentSize();
    const int height = src->GetHeight(plane);
    srcp[0] = src->GetReadPtr(plane);
    maskp[0] = mask->GetWritePtr(plane);

    blur3x3<T>(maskp[0], srcp[0], mask_stride, stride, width, height);

    if (rgb)
    {
        srcp[1] = src->GetReadPtr(PLANAR_B);
        maskp[1] = mask->GetWritePtr(PLANAR_B);
        srcp[2] = src->GetReadPtr(PLANAR_R);
        maskp[2] = mask->GetWritePtr(PLANAR_R);

        blur3x3<T>(maskp[1], srcp[1], mask_stride, stride, width, height);
        blur3x3<T>(maskp[2], srcp[2], mask_stride, stride, width, height);
    }

    int maximum = 0xffff >> (16 - vi.BitsPerComponent());
    int threshold = static_cast<int>((_threshold * maximum) / 100);
    threshold = clamp(threshold, 0, maximum);

    findEdges<T, rgb>((void**)maskp, mask_stride, width, height, threshold, maximum);

    if (rgb)
    {
        if (_mask)
        {
            memcpy(maskp[1], maskp[0], static_cast<int64_t>(height) * mask_stride);
            memcpy(maskp[2], maskp[0], static_cast<int64_t>(height) * mask_stride);
        }
    }
    else
    {
        if (vi.NumComponents() > 1 && (processPlane[1] && processPlane[2]))
        {
            const int strideu = mask->GetPitch(PLANAR_U);
            const int widthu = mask->GetRowSize(PLANAR_U) / vi.ComponentSize();
            const int heightu = mask->GetHeight(PLANAR_U);
            uint8_t* maskpu = mask->GetWritePtr(PLANAR_U);

            copyMask<T>(maskpu, maskp[0], strideu, mask_stride, widthu, heightu, vi.GetPlaneWidthSubsampling(PLANAR_U), vi.GetPlaneHeightSubsampling(PLANAR_U));

            if (_mask)
                memcpy(mask->GetWritePtr(PLANAR_V), mask->GetWritePtr(PLANAR_U), static_cast<int64_t>(heightu) * strideu);
        }
    }
}

template <typename T>
static inline void maskedBlur3x3(void* dstpv, const void* srcpv, const void* maskpv, int dst_stride, int stride, int mask_stride, int width, int height)
{
    T* dstp = (T*)dstpv;
    const T* srcp = (const T*)srcpv;
    const T* maskp = (const T*)maskpv;

    stride /= sizeof(T);
    mask_stride /= sizeof(T);
    dst_stride /= sizeof(T);

    if (!maskp[0]) {
        int count = 1;
        int sum = srcp[0];
        if (!maskp[1]) {
            sum += srcp[1];
            count++;
        }
        if (!maskp[mask_stride]) {
            sum += srcp[stride];
            count++;
        }
        if (!maskp[mask_stride + 1]) {
            sum += srcp[stride + 1];
            count++;
        }

        dstp[0] = static_cast<int>(static_cast<float>(sum) / count + 0.5f);
    }
    else {
        dstp[0] = srcp[0];
    }

    for (int x = 1; x < width - 1; x++) {
        if (!maskp[x]) {
            int count = 1;
            int sum = 0;
            if (!maskp[x - 1]) {
                sum += srcp[x - 1];
                count++;
            }
            sum += srcp[x];
            if (!maskp[x + 1]) {
                sum += srcp[x + 1];
                count++;
            }
            if (!maskp[x + mask_stride - 1]) {
                sum += srcp[x + stride - 1];
                count++;
            }
            if (!maskp[x + mask_stride]) {
                sum += srcp[x + stride];
                count++;
            }
            if (!maskp[x + mask_stride + 1]) {
                sum += srcp[x + stride + 1];
                count++;
            }

            dstp[x] = static_cast<int>(static_cast<float>(sum) / count + 0.5f);
        }
        else {
            dstp[x] = srcp[x];
        }
    }

    if (!maskp[width - 1]) {
        int count = 1;
        int sum = 0;
        if (!maskp[width - 2]) {
            sum += srcp[width - 2];
            count++;
        }
        sum += srcp[width - 1];
        if (!maskp[width + mask_stride - 2]) {
            sum += srcp[width + stride - 2];
            count++;
        }
        if (!maskp[width + mask_stride - 1]) {
            sum += srcp[width + stride - 1];
            count++;
        }

        dstp[width - 1] = static_cast<int>(static_cast<float>(sum) / count + 0.5f);
    }
    else {
        dstp[width - 1] = srcp[width - 1];
    }

    srcp += stride;
    dstp += dst_stride;
    maskp += mask_stride;

    for (int y = 1; y < height - 1; y++) {
        if (!maskp[0]) {
            int count = 1;
            int sum = 0;
            if (!maskp[-mask_stride]) {
                sum += srcp[-stride];
                count++;
            }
            if (!maskp[-mask_stride + 1]) {
                sum += srcp[-stride + 1];
                count++;
            }
            sum += srcp[0];
            if (!maskp[1]) {
                sum += srcp[1];
                count++;
            }
            if (!maskp[mask_stride]) {
                sum += srcp[stride];
                count++;
            }
            if (!maskp[mask_stride + 1]) {
                sum += srcp[stride + 1];
                count++;
            }

            dstp[0] = static_cast<int>(static_cast<float>(sum) / count + 0.5f);
        }
        else {
            dstp[0] = srcp[0];
        }

        for (int x = 1; x < width - 1; x++) {
            if (!maskp[x]) {
                int count = 1;
                int sum = 0;
                if (!maskp[x - mask_stride - 1]) {
                    sum += srcp[x - stride - 1];
                    count++;
                }
                if (!maskp[x - mask_stride]) {
                    sum += srcp[x - stride];
                    count++;
                }
                if (!maskp[x - mask_stride + 1]) {
                    sum += srcp[x - stride + 1];
                    count++;
                }
                if (!maskp[x - 1]) {
                    sum += srcp[x - 1];
                    count++;
                }
                sum += srcp[x];
                if (!maskp[x + 1]) {
                    sum += srcp[x + 1];
                    count++;
                }
                if (!maskp[x + mask_stride - 1]) {
                    sum += srcp[x + stride - 1];
                    count++;
                }
                if (!maskp[x + mask_stride]) {
                    sum += srcp[x + stride];
                    count++;
                }
                if (!maskp[x + mask_stride + 1]) {
                    sum += srcp[x + stride + 1];
                    count++;
                }

                dstp[x] = static_cast<int>(static_cast<float>(sum) / count + 0.5f);
            }
            else {
                dstp[x] = srcp[x];
            }
        }

        if (!maskp[width - 1]) {
            int count = 1;
            int sum = 0;
            if (!maskp[width - mask_stride - 2]) {
                sum += srcp[width - stride - 2];
                count++;
            }
            if (!maskp[width - mask_stride - 1]) {
                sum += srcp[width - stride - 1];
                count++;
            }
            if (!maskp[width - 2]) {
                sum += srcp[width - 2];
                count++;
            }
            sum += srcp[width - 1];
            if (!maskp[width + mask_stride - 2]) {
                sum += srcp[width + stride - 2];
                count++;
            }
            if (!maskp[width + mask_stride - 1]) {
                sum += srcp[width + stride - 1];
                count++;
            }

            dstp[width - 1] = static_cast<int>(static_cast<float>(sum) / count + 0.5f);
        }
        else {
            dstp[width - 1] = srcp[width - 1];
        }

        srcp += stride;
        dstp += dst_stride;
        maskp += mask_stride;
    }

    if (!maskp[0]) {
        int count = 1;
        int sum = 0;
        if (!maskp[-mask_stride]) {
            sum += srcp[-stride];
            count++;
        }
        if (!maskp[-mask_stride + 1]) {
            sum += srcp[-stride + 1];
            count++;
        }
        sum += srcp[0];
        if (!maskp[1]) {
            sum += srcp[1];
            count++;
        }

        dstp[0] = static_cast<int>(static_cast<float>(sum) / count + 0.5f);
    }
    else {
        dstp[0] = srcp[0];
    }

    for (int x = 1; x < width - 1; x++) {
        if (!maskp[x]) {
            int count = 1;
            int sum = 0;
            if (!maskp[x - mask_stride - 1]) {
                sum += srcp[x - stride - 1];
                count++;
            }
            if (!maskp[x - mask_stride]) {
                sum += srcp[x - stride];
                count++;
            }
            if (!maskp[x - mask_stride + 1]) {
                sum += srcp[x - stride + 1];
                count++;
            }
            if (!maskp[x - 1]) {
                sum += srcp[x - 1];
                count++;
            }
            sum += srcp[x];
            if (!maskp[x + 1]) {
                sum += srcp[x + 1];
                count++;
            }

            dstp[x] = static_cast<int>(static_cast<float>(sum) / count + 0.5f);
        }
        else {
            dstp[x] = srcp[x];
        }
    }

    if (!maskp[width - 1]) {
        int count = 1;
        int sum = 0;
        if (!maskp[width - mask_stride - 2]) {
            sum += srcp[width - stride - 2];
            count++;
        }
        if (!maskp[width - mask_stride - 1]) {
            sum += srcp[width - stride - 1];
            count++;
        }
        if (!maskp[width - 2]) {
            sum += srcp[width - 2];
            count++;
        }
        sum += srcp[width - 1];

        dstp[width - 1] = static_cast<int>(static_cast<float>(sum) / count + 0.5f);
    }
    else {
        dstp[width - 1] = srcp[width - 1];
    }
}

static void copy_plane(PVideoFrame& dst, PVideoFrame& src, int plane, IScriptEnvironment* env)
{
    const uint8_t* srcp = src->GetReadPtr(plane);
    int src_pitch = src->GetPitch(plane);
    int height = src->GetHeight(plane);
    int row_size = src->GetRowSize(plane);
    uint8_t* destp = dst->GetWritePtr(plane);
    int dst_pitch = dst->GetPitch(plane);
    env->BitBlt(destp, dst_pitch, srcp, src_pitch, row_size, height);
}

template <typename T>
void vsMSmooth::smooth(PVideoFrame& dst, PVideoFrame& src, PVideoFrame& mask, IScriptEnvironment* env)
{
    int planes_y[3] = { PLANAR_Y, PLANAR_U, PLANAR_V };
    int planes_r[3] = { PLANAR_G, PLANAR_B, PLANAR_R };
    const int* plane = vi.IsRGB() ? planes_r : planes_y;

    if (processPlane[0])
    {
        const int stride = src->GetPitch(plane[0]);
        const int dst_stride = dst->GetPitch(plane[0]);
        const int mask_stride = mask->GetPitch(plane[0]);
        const int width = src->GetRowSize(plane[0]) / vi.ComponentSize();
        const int height = src->GetHeight(plane[0]);
        const uint8_t* srcp = src->GetReadPtr(plane[0]);
        const uint8_t* maskp = mask->GetReadPtr(plane[0]);
        uint8_t* dstp = dst->GetWritePtr(plane[0]);

        maskedBlur3x3<T>(dstp, srcp, maskp, dst_stride, stride, mask_stride, width, height);
    }
    else
        copy_plane(dst, src, plane[0], env);

    if (vi.NumComponents() > 1)
    {
        const uint8_t* maskpu = mask->GetReadPtr(vi.IsRGB() ? plane[0] : plane[1]);

        if (processPlane[1])
        {
            const int strideu = src->GetPitch(plane[1]);
            const int dst_strideu = dst->GetPitch(plane[1]);
            const int mask_strideu = mask->GetPitch(plane[1]);
            const int widthu = src->GetRowSize(plane[1]) / vi.ComponentSize();
            const int heightu = src->GetHeight(plane[1]);
            const uint8_t* srcpu = src->GetReadPtr(plane[1]);
            uint8_t* dstpu = dst->GetWritePtr(plane[1]);

            maskedBlur3x3<T>(dstpu, srcpu, maskpu, dst_strideu, strideu, mask_strideu, widthu, heightu);
        }
        else
            copy_plane(dst, src, plane[1], env);

        if (processPlane[2])
        {
            const int stridev = src->GetPitch(plane[2]);
            const int dst_stridev = dst->GetPitch(plane[2]);
            const int mask_stridev = mask->GetPitch(plane[2]);
            const int widthv = src->GetRowSize(plane[2]) / vi.ComponentSize();
            const int heightv = src->GetHeight(plane[2]);
            const uint8_t* srcpv = src->GetReadPtr(plane[2]);
            uint8_t* dstpv = dst->GetWritePtr(plane[2]);

            maskedBlur3x3<T>(dstpv, srcpv, maskpu, dst_stridev, stridev, mask_stridev, widthv, heightv);
        }
        else
            copy_plane(dst, src, plane[2], env);
    }
}

vsMSmooth::vsMSmooth(PClip _child, double threshold, double strength, bool mask, bool luma, bool chroma, IScriptEnvironment* env)
    : GenericVideoFilter(_child), _threshold(threshold), _strength(strength), _mask(mask), _luma(luma), _chroma(chroma)
{
    if (!vi.IsPlanar())
        env->ThrowError("vsMSmooth: Clip must be in planar format.");

    if (vi.BitsPerComponent() == 32)
        env->ThrowError("vsMSmooth: Only 8..16 bit integer input supported.");

    if (_threshold < 0.0 || _threshold > 100.0)
        env->ThrowError("vsMSmooth: threshold must be between 0 and 100%.");

    if (_strength < 1.0 || _strength > 25.0)
        env->ThrowError("vsMSmooth: strength must be between 1 and 25 (inclusive).");

    has_at_least_v8 = true;
    try { env->CheckVersion(8); }
    catch (const AvisynthError&) { has_at_least_v8 = false; }

    memset(processPlane, 0, sizeof(processPlane));

    int planecount = min(vi.NumComponents(), 3);
    for (int i = 0; i < planecount; i++)
    {
        if (vi.IsPlanarRGB())
            processPlane[i] = true;
        else if (i == 0)
            processPlane[i] = _luma;
        else
            processPlane[i] = _chroma;
    }
}

PVideoFrame __stdcall vsMSmooth::GetFrame(int n, IScriptEnvironment* env)
{
    PVideoFrame src = child->GetFrame(n, env);
    PVideoFrame mask = env->NewVideoFrame(vi);
    PVideoFrame temp = env->NewVideoFrame(vi);
    PVideoFrame dst = env->NewVideoFrame(vi);

    if (vi.IsRGB())
    {
        if (vi.BitsPerComponent() == 8)
            edgeMask<uint8_t, true>(mask, src, env);
        else
            edgeMask<uint16_t, true>(mask, src, env);
    }
    else
    {
        if (vi.BitsPerComponent() == 8)
            edgeMask<uint8_t, false>(mask, src, env);
        else
            edgeMask<uint16_t, false>(mask, src, env);
    }

    if (_mask)
    {
        if (has_at_least_v8)
            env->copyFrameProps(src, mask);

        return mask;
    }

    if (vi.BitsPerComponent() == 8)
        smooth<uint8_t>(dst, src, mask, env);
    else
        smooth<uint16_t>(dst, src, mask, env);

    for (int i = 1; i < static_cast<int>(_strength + 0.5); i++)
    {
        temp = dst;
        dst = env->NewVideoFrame(vi);

        if (vi.BitsPerComponent() == 8)
            smooth<uint8_t>(dst, temp, mask, env);
        else
            smooth<uint16_t>(dst, temp, mask, env);
    }

    if (has_at_least_v8)
        env->copyFrameProps(src, dst);

    return dst;
}

AVSValue __cdecl Create_vsMSmooth(AVSValue args, void* user_data, IScriptEnvironment* env)
{
    return new vsMSmooth(
        args[0].AsClip(),
        args[1].AsFloat(6.0),
        args[2].AsFloat(3.0),
        args[3].AsBool(false),
        args[4].AsBool(true),
        args[5].AsBool(false),
        env);
}

const AVS_Linkage* AVS_linkage;

extern "C" __declspec(dllexport)
const char* __stdcall AvisynthPluginInit3(IScriptEnvironment * env, const AVS_Linkage* const vectors)
{
    AVS_linkage = vectors;

    env->AddFunction("vsMSmooth", "c[threshold]f[strength]f[mask]b[luma]b[chroma]b", Create_vsMSmooth, 0);
    return "vsMSmooth";
}
