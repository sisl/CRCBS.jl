using CRCBS
using Documenter

makedocs(;
    modules=[CRCBS],
    authors="kylebrown <kylejbrown17@gmail.com> and contributors",
    repo="https://github.com/kylejbrown17/CRCBS.jl/blob/{commit}{path}#L{line}",
    sitename="CRCBS.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://kylejbrown17.github.io/CRCBS.jl",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
    ],
)

deploydocs(;
    repo="github.com/kylejbrown17/CRCBS.jl",
)
