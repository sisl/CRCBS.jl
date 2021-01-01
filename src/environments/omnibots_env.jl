# """
#     TimeStampedState{S,T}
#
# Wrapper for states that contain a time stamp
# """
# struct TimeStampedState{S,T}
#     s::S
#     t::T
# end
# struct CountdownState{S}
#     s::S
#     counter::Int
# end

export
    OmniBotState,
    OmniBotAction

"""
    AbstractOmniBotState{N}

Abstract type representing the state of an omnidirectional robot.
"""
abstract type AbstractOmniBotState{N} end
get_t(s::AbstractOmniBotState) = s.t
Base.show(io::IO,s::AbstractOmniBotState) = print(io,
    typeof(s),"(\n  pos=",[s.pos...],",\n  vel=",[s.vel...],",\n  θ=",s.θ,"\n  ω=",s.ω)

"""
    OmniBotState{N}

Represents the state of an omnidirectional robot. Assumes that the robot can
only rotate along one axis.
Fields:
- position `pos`
- velocity `vel`
- angular orientation `θ`
- angular velocity `ω`
"""
@with_kw_noshow struct OmniBotState{N} <: AbstractOmniBotState{N}
    pos::SVector{N,Float64} = zero(SVector{N,Float64})
    vel::SVector{N,Float64} = zero(SVector{N,Float64})
    θ::Float64              = 0.0
    ω::Float64              = 0.0
    t::Float64              = 0.0
end
# struct PlatformOmniBotState{N} <: AbstractOmniBotState{N}
#     pos::SVector{N,Float64}
#     vel::SVector{N,Float64}
#     θ::Float64
#     ω::Float64
#     platform_height::Float64
# end

"""
    AbstractOmniBotAction{N}

Abstract type representing the action of an omnidirectional robot.
"""
abstract type AbstractOmniBotAction{N} end
get_dt(a::AbstractOmniBotAction)  = a.dt
Base.show(io::IO,a::AbstractOmniBotAction) = print(io,
    typeof(a),"(\n  accel=",[a.accel...],",\n  α=",a.α,")")

"""
    OmniBotAction{N}

Represents the action of an omnidirectional robot. Assumes that the robot can
Fields:
- positional acceleration `accel`
- angular acceleration `α`
"""
@with_kw_noshow struct OmniBotAction{N} <: AbstractOmniBotAction{N}
    accel::SVector{N,Float64}   = zero(SVector{N,Float64})
    α::Float64                  = 0.0
    dt::Float64                 = 1.0
end
function get_next_state(s::OmniBotState,a::AbstractOmniBotAction)
    OmniBotState(
        pos = s.pos .+ s.vel * a.dt .+ a.accel * 0.5 * a.dt^2,
        vel = s.vel .+ a.accel * a.dt,
        θ   = s.θ + s.ω * a.dt .+ a.α * 0.5 * a.dt^2,
        ω   = s.ω + a.α * a.dt,
        t   = s.t + a.dt
    )
end

abstract type AbstractOmniBotModel end
# """
#     OmniBotModel{N}
#
# Describes an OmniBotModel.
# """
# @with_kw_noshow struct OmniBotModel{N} <: AbstractOmniBotModel
#     ϵ::Float64 = 1e-4
#     v_max::SVector{N,Float64} = SVector(1.0,1.0,0.0)
#     v_min::SVector{N,Float64} = SVector(-1.0,-1.0,0.0)
#     ω_max::Float64 = π/2
#     ω_min::Float64 = -π/2
#     # s_max::OmniBotState{N} = OmniBotState{N}(
#     #     pos=SVector(Inf,Inf,Inf),
#     #     vel=SVector(1.0,1.0,0.0),
#     #     θ=Inf,
#     #     ω=π/4.0,
#     # )
#     # s_min::OmniBotState{N} = OmniBotState{N}(
#     #     pos=SVector(-Inf,-Inf,-Inf),
#     #     vel=SVector(-1.0,-1.0,0.0),
#     #     θ=-Inf,
#     #     ω=-π/4.0,
#     # )
#     # a_max::OmniBotAction{N} = OmniBotAction{N}(
#     #     accel=SVector(1.0,1.0,1.0),
#     #     α=SVector(1.0,1.0,1.0),
#     # )
#     # max_vel::SVector{N,Float64}     = SVector(1.0,1.0,0.0)
#     # max_ω::Float64                  = SVector(1.0,1.0,0.0)
#     # dt::Float64                     = 1.0
#     # accel_steps::SVector{N,Float64} = SVector(1.0/dt,1.0/dt,0.0)
#     # α_step::Float64                 = π/(2*dt)
# end

@with_kw_noshow struct BangBangOmnibotActionSpace{N}
    a_max::OmniBotAction{N} = OmniBotAction{N}(
        accel=SVector(1.0,1.0,0.0),
        α=π/4,
    )
    a_min::OmniBotAction{N} = OmniBotAction{N}(
        accel=SVector(-1.0,-1.0,0.0),
        α=-π/4,
    )
    dt::Float64 = 1,0
end

"""
    discretized_omnibot_model()

Defines an `OmniBotModel` with a discretized "bang-bang" action space.
"""
function discretized_omnibot_model(
    vel_steps=(0.5,0.5,0),
    ω_bins=3,
    θ_step=π/4,
    )

end

"""
    get_possible_actions(model,s::OmniBotState{N}) where {N}

The robot may only rotate if it has no cartesian velocity. It can only
accelerate +/- the maximum accel, and must remain within velocity bounds. It can
only accelerate laterally if not already moving longitudinally, and vice versa.
"""
function get_possible_actions(model,s::OmniBotState{N}) where {N}
    accels = Vector{SVector{N,Float64}}([[0.0,0.0,0.0]])
    α_list = [0.0]
    if abs(s.vel[2]) < model.ϵ && abs(s.ω) < model.ϵ
        if s.vel[1] + model.ϵ < model.v_max[1]
            push!(accels, [1.0,0.0,0.0])
        end
        if s.vel[1] - model.ϵ > model.v_min[1]
            push!(accels, [-1.0,0.0,0.0])
        end
    end
    if abs(s.vel[1]) < model.ϵ
        if s.vel[2] + model.ϵ < model.v_max[2]
            push!(accels, [0.0,1.0,0.0])
        end
        if s.vel[2] - model.ϵ > model.v_min[2]
            push!(accels, [0.0,-1.0,0.0])
        end
    end
    if abs(s.vel[1]) < model.ϵ && abs(s.vel[2]) < model.ϵ
        if s.ω + model.ϵ < model.ω_max
            push!(α_list, 1.0*model.α_step)
        end
        if s.ω - model.ϵ > model.ω_min
            push!(α_list, -1.0*model.α_step)
        end
    end
    actions = Vector{OmniBotAction{N}}()
    for accel in accels
        if norm(accel) < model.ϵ
            for α in α_list
                push!(actions,OmniBotAction{N}(accel=accel,α=α,dt=model.dt))
            end
        else
            α = 0.0
            push!(actions,OmniBotAction{N}(accel=accel,α=α,dt=model.dt))
        end
    end
    return actions
end
