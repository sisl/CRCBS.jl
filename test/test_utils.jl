let
    v = [0,1,2,4,5,6]
    L = length(v)
    x = 3
    @test find_index_in_sorted_array(v,x) == 4
    insert_to_sorted_array!(v,x)
    @test length(v) == L + 1
    v[4] == x
end
let
    v = [0,1,1,1,1,2]
    L = length(v)
    x = 1
    @test find_index_in_sorted_array(v,x) == 2
end
let
    v = Vector{Int}()
    insert_to_sorted_array!(v,1)
    @test length(v) == 1
end
