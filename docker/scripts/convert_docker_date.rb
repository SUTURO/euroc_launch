require 'time'

# Usage:
# Pipe the date to convert into this script
# Example: 
# $ ruby get_image_stat.rb suturo data created | ruby convert_docker_date.rb
#   2015-03-31 15:13:18 UTC

input = ARGF.read

# For each line of the input...
input.each_line do |line|
	date = Time.parse line
	puts date.strftime("%d.%m.%Y %T")
end

