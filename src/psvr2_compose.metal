// The PS VR2 compositor: for every pixel of the 4000x2040 panel, where it looks through the lens
// (the headset's own calibration and Monado's table, as psvr2-player-mac corrects it), turned by
// the head's rotation since the game rendered, and that ray's place in the game's eye image.
#include <metal_stdlib>
using namespace metal;

struct VOut { float4 pos [[position]]; float2 uv; };

vertex VOut vs(uint vid [[vertex_id]]) {
    float2 p[3] = { float2(-1.0, -1.0), float2(3.0, -1.0), float2(-1.0, 3.0) };
    VOut o;
    o.pos = float4(p[vid], 0.0, 1.0);
    o.uv = float2(p[vid].x * 0.5 + 0.5, 1.0 - (p[vid].y * 0.5 + 0.5));   // (0,0) top left
    return o;
}

struct Uni {
    float4 calibL;     // k1, k2, k3, k4 of each eye, from the headset (psvr2_get_distortion_calibration)
    float4 calibR;
    float4x4 delta;    // a direction in the head now -> in the head the game rendered for
    float4 p0;         // x: a game frame is there, y: chromatic correction
};

// The game's eye images cover this much of tan space: x from -cx / FX to (1 - cx) / FX, y from
// -0.5 / FY to 0.5 / FY. The runtime reports the same field of view (backend_psvr2.m).
constant float FX = 0.3585564;
constant float FY = 0.3762281;

fragment float4 fs(VOut in [[stage_in]], constant Uni &u [[buffer(0)]],
                   device const packed_float3 *lut [[buffer(1)]],
                   texture2d<float> left [[texture(0)]], texture2d<float> right [[texture(1)]]) {
    constexpr sampler s(filter::linear, address::clamp_to_edge);
    int eye = in.uv.x < 0.5 ? 0 : 1;
    float fU = eye == 0 ? in.uv.x * 2.0 : (in.uv.x - 0.5) * 2.0;
    float fV = in.uv.y;

    float4 c = eye == 0 ? u.calibL : u.calibR;
    float k1 = c.x, k2 = c.y, k3 = c.z, k4 = c.w;
    float local_x = k1 + (fU * 2.0 - 1.0) * 0.9803922 * 0.9394987;
    float local_y = k2 + (fV * 2.0 - 1.0) * 0.9394987;

    float Rs = (local_x * local_x + local_y * local_y) * 1023.0;
    int idx = int(Rs);
    float3 scale;
    if (idx > -1 && idx + 1 < 1024) {
        float frac = Rs - float(idx);
        scale = frac * float3(lut[idx + 1]) + (1.0 - frac) * float3(lut[idx]);
    } else {
        float v = Rs - 1023.0;
        scale = float3(v * 0.003648639 + 1.592586, v * 0.004862547 + 1.692979, v * 0.007425308 + 1.841771);
    }

    if (u.p0.x < 0.5) { return float4(0.08, 0.08, 0.08, 1.0); }   // no game frame yet

    float cx = eye == 0 ? 0.6603788 : 0.3396213;
    bool chromatic = u.p0.y > 0.5;
    int from = chromatic ? 0 : 1, to = chromatic ? 2 : 1;
    float3 rgb = float3(0.0);
    for (int ch = from; ch <= to; ch++) {
        float a = local_x * scale[ch];
        float b = (local_y + (ch == 1 ? -0.0002302693 : 0.0)) * scale[ch];
        float tanx = k3 * a - k4 * b;
        float tanyDown = k4 * a + k3 * b;
        float3 d = (u.delta * float4(tanx, -tanyDown, -1.0, 0.0)).xyz;
        if (d.z > -1e-3) { continue; }
        float2 uv = float2(d.x / -d.z * FX + cx, -d.y / -d.z * FY + 0.5);
        if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) { continue; }
        float3 sampled = (eye == 0 ? left.sample(s, uv) : right.sample(s, uv)).rgb;
        if (chromatic) { rgb[ch] = sampled[ch]; } else { rgb = sampled; }
    }
    return float4(rgb, 1.0);
}
