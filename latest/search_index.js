var documenterSearchIndex = {"docs": [

{
    "location": "#",
    "page": "Home",
    "title": "Home",
    "category": "page",
    "text": ""
},

{
    "location": "#CRCBS.jl-Documentation-1",
    "page": "Home",
    "title": "CRCBS.jl Documentation",
    "category": "section",
    "text": "Welcome to the CRCBS.jl documentation."
},

{
    "location": "#Making-A-Custom-Package-Logo-1",
    "page": "Home",
    "title": "Making A Custom Package Logo",
    "category": "section",
    "text": "To make a custom package logo and include it in the documentation, simply add a logo.png file to /docs/assets/. For the logo to render well, it is recommended to be 500px by 500px and at least 144dpi."
},

{
    "location": "modules/CRCBS/#",
    "page": "CRCBS",
    "title": "CRCBS",
    "category": "page",
    "text": ""
},

{
    "location": "modules/CRCBS/#CRCBS.GraphPath",
    "page": "CRCBS",
    "title": "CRCBS.GraphPath",
    "category": "type",
    "text": "Type alias for a path through the graph \n\n\n\n\n\n"
},

{
    "location": "modules/CRCBS/#CRCBS.traversal_time",
    "page": "CRCBS",
    "title": "CRCBS.traversal_time",
    "category": "function",
    "text": "returns the time for traversal of a GraphPath. Defaults to the length of the\npath, but it may be useful in case we need to override later\n\n\n\n\n\n"
},

{
    "location": "modules/CRCBS/#CRCBS.LowLevelSolution",
    "page": "CRCBS",
    "title": "CRCBS.LowLevelSolution",
    "category": "type",
    "text": "Type alias for a list of agent paths \n\n\n\n\n\n"
},

{
    "location": "modules/CRCBS/#CRCBS.MAPF",
    "page": "CRCBS",
    "title": "CRCBS.MAPF",
    "category": "type",
    "text": "A MAPF is an instance of a Multi Agent Path Finding problem. It consists of\na graph `G` whose edges have unit length, as well as a set of start and goal\nvertices on that graph. Note that this is the _labeled_ case, where each\nagent has a specific assigned destination.\n\n\n\n\n\n"
},

{
    "location": "modules/CRCBS/#CRCBS.NodeConflict",
    "page": "CRCBS",
    "title": "CRCBS.NodeConflict",
    "category": "type",
    "text": "Encodes a conflict wherein two agents occupy a particular node at a\nparticular time\n\n\n\n\n\n"
},

{
    "location": "modules/CRCBS/#CRCBS.invalid_node_conflict",
    "page": "CRCBS",
    "title": "CRCBS.invalid_node_conflict",
    "category": "function",
    "text": "Returns an invalid NodeConflict \n\n\n\n\n\n"
},

{
    "location": "modules/CRCBS/#CRCBS.EdgeConflict",
    "page": "CRCBS",
    "title": "CRCBS.EdgeConflict",
    "category": "type",
    "text": "Encodes a conflict between two agents at a particular edge at a particular\ntime. This means that the agents are trying to swap places at time t.\n\n\n\n\n\n"
},

{
    "location": "modules/CRCBS/#CRCBS.invalid_edge_conflict",
    "page": "CRCBS",
    "title": "CRCBS.invalid_edge_conflict",
    "category": "function",
    "text": "Returns an invalid EdgeConflict \n\n\n\n\n\n"
},

{
    "location": "modules/CRCBS/#CRCBS.is_valid",
    "page": "CRCBS",
    "title": "CRCBS.is_valid",
    "category": "function",
    "text": "checks if a solution is valid\n\n\n\n\n\nChecks if a node conflict is valid \n\n\n\n\n\nchecks if an edge node is invalid \n\n\n\n\n\n"
},

{
    "location": "modules/CRCBS/#CRCBS.CBSConstraint",
    "page": "CRCBS",
    "title": "CRCBS.CBSConstraint",
    "category": "type",
    "text": "Encodes a constraint that agent `a` may not occupy vertex `v` at time `t`\n\n\n\n\n\n"
},

{
    "location": "modules/CRCBS/#CRCBS.ConstraintTreeNode",
    "page": "CRCBS",
    "title": "CRCBS.ConstraintTreeNode",
    "category": "type",
    "text": "A node of a constraint tree. Each node has a set of constraints, a candidate\nsolution (set of robot paths), and a cost\n\n\n\n\n\n"
},

{
    "location": "modules/CRCBS/#CRCBS.ConstraintDict",
    "page": "CRCBS",
    "title": "CRCBS.ConstraintDict",
    "category": "type",
    "text": "constraint dictionary for fast constraint lookup within a_star\n\n\n\n\n\n"
},

{
    "location": "modules/CRCBS/#CRCBS-1",
    "page": "CRCBS",
    "title": "CRCBS",
    "category": "section",
    "text": "GraphPath\ntraversal_time\nLowLevelSolution\nMAPF\nNodeConflict\ninvalid_node_conflict\nEdgeConflict\ninvalid_edge_conflict\nis_valid\nCBSConstraint\nConstraintTreeNode\nConstraintDict"
},

{
    "location": "modules/rubber_ducks/#",
    "page": "Rubber Ducks",
    "title": "Rubber Ducks",
    "category": "page",
    "text": ""
},

{
    "location": "modules/rubber_ducks/#Rubber-Ducks-1",
    "page": "Rubber Ducks",
    "title": "Rubber Ducks",
    "category": "section",
    "text": "You can modify the name of both the file in the documentationrubber_ducks_in_earth\nRUBBER_DUCK_PER_SEC"
},

{
    "location": "examples/plotting_example/#",
    "page": "Plotting Example",
    "title": "Plotting Example",
    "category": "page",
    "text": ""
},

{
    "location": "examples/plotting_example/#Plotting-Example-1",
    "page": "Plotting Example",
    "title": "Plotting Example",
    "category": "section",
    "text": "In this example we showcase how to automatically build and generate plots as part of the documentation deployment process.The plots in this example are produced by the file docks/src/makeplots.jl The contents of that file are run each time the documentation is deployed. (Image: )note: Note\nPlots will only appear in documentation when deployed to github pages.warning: Orange Box\nHere is how you add warning box to your documentation"
},

{
    "location": "library_index/#",
    "page": "Library Index",
    "title": "Library Index",
    "category": "page",
    "text": ""
},

]}
