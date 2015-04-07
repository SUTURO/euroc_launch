require 'rubygems'
require 'json'
require 'rest-client' 


# This script fetches information from a docker registry.
# It requires the namespace and repository name of a docker repo plus
# the name of the image attribute.
# The image that will be looked up from the given repo, is always the "latest"
# 
# Example output for detailled image information:
# {"id"=>"a0203b18aaae01d5cdb03184896829b9af556c8c70d77ddb6ceee0670ea3cb39", "parent"=>"95e4eb89ab1e9787d56615830e748a3a457dee21c0679e208d316c83e62cab2d", "created"=>"2015-03-31T15:13:18.49895885Z", "container"=>"be262e04b8cc53990bde14eb3754cc509d4b26f47e99325c7042d9dd7c2b1979", "container_config"=>{"Hostname"=>"e564850f8fd8", "Domainname"=>"", "User"=>"suturo", "Memory"=>0, "MemorySwap"=>0, "CpuShares"=>0, "Cpuset"=>"", "AttachStdin"=>false, "AttachStdout"=>false, "AttachStderr"=>false, "PortSpecs"=>nil, "ExposedPorts"=>nil, "Tty"=>false, "OpenStdin"=>false, "StdinOnce"=>false, "Env"=>["HOME=/", "PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"], "Cmd"=>["/bin/sh", "-c", "#(nop) USER [suturo]"], "Image"=>"95e4eb89ab1e9787d56615830e748a3a457dee21c0679e208d316c83e62cab2d", "Volumes"=>{"/data/db"=>{}, "/home/suturo/.cache"=>{}, "/home/suturo/.rviz"=>{}}, "WorkingDir"=>"", "Entrypoint"=>nil, "NetworkDisabled"=>false, "MacAddress"=>"", "OnBuild"=>[]}, "docker_version"=>"1.5.0", "config"=>{"Hostname"=>"e564850f8fd8", "Domainname"=>"", "User"=>"suturo", "Memory"=>0, "MemorySwap"=>0, "CpuShares"=>0, "Cpuset"=>"", "AttachStdin"=>false, "AttachStdout"=>false, "AttachStderr"=>false, "PortSpecs"=>nil, "ExposedPorts"=>nil, "Tty"=>false, "OpenStdin"=>false, "StdinOnce"=>false, "Env"=>["HOME=/", "PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"], "Cmd"=>["/bin/bash"], "Image"=>"95e4eb89ab1e9787d56615830e748a3a457dee21c0679e208d316c83e62cab2d", "Volumes"=>{"/data/db"=>{}, "/home/suturo/.cache"=>{}, "/home/suturo/.rviz"=>{}}, "WorkingDir"=>"", "Entrypoint"=>nil, "NetworkDisabled"=>false, "MacAddress"=>"", "OnBuild"=>[]}, "architecture"=>"amd64", "os"=>"linux", "Size"=>0}
# 

unless ARGV.size == 3
	puts "Please pass exactly three parameters like this:
	\nget_image_stat.rb namespace repository name_of_image_attribute.\n"
	exit 1
end

namespace = ARGV[0]
repository = ARGV[1]
base_url = "https://team.suturo.de:9420"
url = "#{base_url}/v1/repositories/#{namespace}/#{repository}/tags"

repo_tags = JSON.parse( RestClient::Request.execute(:url => url, :method => :get, :verify_ssl => false) )

# puts repo_tags.inspect
image_id = repo_tags["latest"]
image_url = "#{base_url}/v1/images/#{image_id}/json"

image_info = JSON.parse( RestClient::Request.execute(:url => image_url, :method => :get, :verify_ssl => false) )
puts image_info[ARGV[2]]

# date = Time.parse image_info[ARGV[2]]
# puts date
