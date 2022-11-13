using CRCBS
using Documenter

makedocs(;
    modules=[CRCBS,CRCBS.BenchmarkInterface],
    authors="kylebrown <kylejbrown17@gmail.com> and contributors",
    repo="https://github.com/sisl/CRCBS.jl",
    sitename="CRCBS.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://kylejbrown17.github.io/CRCBS.jl",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
        "Getting Started" => "getting_started.md",
        "Core Types and Methods" => "library.md",
        "API Reference" => "reference.md",
    ],
)

deploydocs(;
    repo="github.com/kylejbrown17/CRCBS.jl",
)
