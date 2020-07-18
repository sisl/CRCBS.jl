using CairoMakie
CairoMakie.activate!()
using Makie
using GraphUtils

function bbox_corners(pts,pad=0.0)
    c = [minimum(grid_vtxs) .- pad, maximum(grid_vtxs) .+ pad]
    [
        [c[1][1],c[1][2]],
        [c[1][1],c[2][2]],
        [c[2][1],c[2][2]],
        [c[2][1],c[1][2]],
    ]
end
function env_state_snapshot(paths::Vector{Vector{T}},t,scale=1,offset=[0.0,0.0]) where {T}
    positions = map(p -> offset .+ scale*interpolate(p,t), paths)
end
function plot_grid_world!(scene,grid_vtxs)
    x_grid = map(v->v[1],grid_vtxs)
    y_grid = map(v->v[2],grid_vtxs)
    scene = scatter!(scene, x_grid, y_grid, marker=:rect, markersize=1.25, color=:white)
end
function plot_positions!(scene,paths,t)
    positions = env_state_snapshot(paths,t)
    x = map(p->p[1],positions)
    y = map(p->p[2],positions)
    scatter!(scene, x, y, marker=:circle, markersize=1, color=:red)
end
function plot_positions(paths,t)
    positions = env_state_snapshot(paths,t)
    x = map(p->p[1],positions)
    y = map(p->p[2],positions)
    scatter(x, y, marker=:circle, markersize=1, color=:red)
end
function plot_solution!(scene,grid_vtxs,paths,t)
    scene = plot_grid_world!(scene,grid_vtxs)
    scene = plot_positions!(scene,paths,t)
end
function update_solution!(scene,paths,t)
    positions = env_state_snapshot(paths,t)
    x = map(p->p[1],positions)
    y = map(p->p[2],positions)

    plt = scene.plots[end]
    plt.input_args[1][] = x
    plt.input_args[2][] = y
    scene
end

# scene = Scene()
scene = Scene(show_axis=false)
# background
grid_vtxs = [[1,1],[1,2],[2,1],[2,2]]
paths = [
    [[1,1],[1,2],[2,2]],
    [[2,2],[2,1],[1,1]]
    ]
t = 0.0
pad = 0.75
corners = bbox_corners(grid_vtxs,pad)
poly!(scene,Point2f0.(corners), color=:gray)
scene = plot_solution!(scene,grid_vtxs,paths,t)

# xlims!(scene,(0.5,2.5))
# ylims!(scene,(0.5,2.5))
# scene
# axis = scene[Axis]
# axis[:showticks] = false
# axis[:showtickmarks] = false
# scene[Axis].attributes[:visible] = false

record(scene, "robots_moving.mp4", 0:0.1:4.0; framerate = 20) do dt
    update_solution!(scene,paths,t+dt)
end
