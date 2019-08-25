using Documenter,CRCBS

#
include("src/makeplots.jl")

# This function builds the documentation
makedocs(
    modules   = [CRCBS],
    doctest   = false,
    clean     = true,
    linkcheck = false,
    format    = Documenter.HTML(),
    sitename  = "CRCBS.jl",
    authors   = "Kyle Brown",
    pages     = Any[
        "Home" => "index.md",
        "Modules" => Any[
            "modules/CRCBS.md", # Use default module name in sidebar
            "Intialization" => "modules/submodule.md",
        ],
        "Examples" => Any[
            "Plotting Example" => "examples/plotting_example.md"
        ],
        "Library Index" => "library_index.md",
    ]
)

# Generate plots
# Note: Must be called after makedocs so the build folder are created
makeplots()

deploydocs(
    repo = "github.com/kylejbrown17/CRCBS.jl",
    devbranch = "develop",
    devurl = "latest",
    deps = makeplots,
)
