#!/usr/bin/ruby

file=ARGV.first
lines=[]
File.open(file,'r') do |f|
  lines=f.readlines
end

tilex=0.625
tiley=1.25
xtiles=9
ytiles=4
errors=[]
error_sum=0
lines.each do |l|
  line=l.split
  inx,iny = line[5].to_f, line[6].to_f
  trx,try = inx * xtiles, iny * ytiles
  idx,idy = trx.round, try.round
  diff = Math.sqrt(((trx - idx) * tilex) ** 2 + ((try - idy) * tiley) ** 2)
  errors << diff
  error_sum += diff
  puts "in #{inx} #{iny} out #{idx} #{idy} tr #{trx} #{try} diff #{diff}"
end

max_error=errors.max * 100
avg_error=error_sum/errors.size * 100
STDERR.puts "max: #{max_error} avg: #{avg_error} [cm]"
