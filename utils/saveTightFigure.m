function saveTightFigure(h,outfilename,suffix,resolution,varargin)

set(h,'PaperUnits','centimeters');

a=get(h,'Children');

legendIndex = zeros(length(a),1);
for i = 1:length(a)
  if(strcmp(get(a(i),'Tag'),'legend'))
    legendIndex(i) = 1;
  end
end
a(legendIndex==1) = [];
nfigs=length(a);

bounds=zeros(nfigs+1,4);
bounds(end,1:2)=inf;
bounds(end,3:4)=-inf;

for i=1:nfigs
    set(a(i),'Unit','centimeters');
    pos=get(a(i),'Position');
    inset=get(a(i),'OuterPosition');
    bounds(i,:)=[inset(1) inset(2) inset(1)+inset(3), inset(2)+inset(4)];
end

auxmin=min(bounds(:,1:2));
auxmax=max(bounds(:,3:4));
mypos=[auxmin auxmax-auxmin];

if size(varargin,2)==1
    mypos(3)=mypos(3)+varargin{1};
    mypos(4)=mypos(4)+varargin{1};
elseif size(varargin,2)==2
    mypos(3)=mypos(3)+varargin{1};
    mypos(4)=mypos(4)+varargin{2};
end

set(h,'PaperSize',[mypos(3) mypos(4)]);
set(h,'PaperPositionMode', 'manual');
set(h,'PaperPosition',mypos);

print([outfilename '.' suffix],['-d' suffix],['-r' num2str(resolution)]);
