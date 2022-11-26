using CRCBS
using Documenter

makedocs(
    modules   = [CRCBS,CRCBS.BenchmarkInterface],  
    format    = Documenter.HTML(),
    sitename  = "CRCBS.jl",
    pages     = [
        "Home" => "index.md",
        "Getting Started" => "getting_started.md",
        "Core Types and Methods" => "library.md",
        "API Reference" => "reference.md",
    ],
)

# Generate plots
# Note: Must be called after makedocs so the build folder are created
#makeplots()

deploydocs(
    repo = "https://github.com/sisl/CRCBS.jl",
    devbranch = "master",
    devurl = "latest",
    #deps = makeplots,
)