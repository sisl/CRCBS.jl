let
    OmniBotState{2}()
    OmniBotAction{2}()
    OmniBotState{3}()
    OmniBotAction{3}()
end
let
    s = OmniBotState{2}(
        pos = [0,0],
        vel = [0.0,0.0],
    )
    a = OmniBotAction{2}(
        accel = [1.0,0.0],
        dt = 1.0
    )
    sp = get_next_state(s,a)
end
let
    model = (
        ϵ = 1e-4,
        v_max = [1.0,1.0,0.0],
        v_min = [-1.0,-1.0,0.0],
        ω_max = π/2,
        ω_min = -π/2,
        α_step = π/2,
        dt = 1.0,
    )
    model
    s = OmniBotState{3}()
    actions = get_possible_actions(model,s)
    for a in actions
        sp = get_next_state(s,a)
        @show a.α,sp.ω,norm(sp.vel), length(get_possible_actions(model,sp))
    end
end
