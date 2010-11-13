function correctedData = testICP(anglex, angley, anglez)
  % erstellung der testrotation, Winkelangabe in grad
  Rz = [cos(anglez) -sin(anglez) 0;
        sin(anglez)  cos(anglez) 0;
        0           0          1];

  Ry = [  cos(angley) 0  sin(angley);
          0          1  0;
         -sin(angley) 0  cos(angley)];

  Rx = [1  0           0;
        0  cos(anglex) -sin(anglex);
        0  sin(anglex)  cos(anglex)];

  R = Rz * Ry * Rx;
  
  %% generate random model
  model = rand(3, 10)*100
  
  %% generate noise
  noise = rand(3, 10)*5;
  
  % rotate and translate model to use as data to be matched
  %data = R * model + 200;

  % rotate and translate model to use as data to be matched and add noise
  data = R * model + 200 + noise;
  
  
  %% center model cloud
  cm = sum(model')' / size(model,2);
  centerModel = model - repmat(cm, 1, size(model,2)); 

  %% center data cloud
  cd = sum(data')' / size(data,2);
  centerData = data - repmat(cd, 1, size(data,2)); 
      
  %% correlation matrix H
  H = zeros(3,3);
  for i=1:size(data,2)
    for j=1:3
      for k=1:3
        H(j,k) = H(j,k) + centerModel(j,i)*centerData(k,i);
      end
    end
  end

%  H = zeros(3,3);
%  for i=1:3
%    for j=1:3
%      H(i,j) = model(i,:) *data(j,:)';
%    end
%  end
    
  %% rotation
  % error in the paper: not H*H' but H'*H!
  HH = H'*H;
  
  [eVec, eVal] = eig(HH);
  
  lamb1=eVal(1,1);
  lamb2=eVal(2,2);
  lamb3=eVal(3,3);
  
  u1=eVec(:,1);
  u2=eVec(:,2);
  u3=eVec(:,3);
  
  R=H*(1/sqrt(lamb1)*(u1*u1') + 1/sqrt(lamb2)*(u2*u2') + 1/sqrt(lamb3)*(u3*u3'))
  
  
  %% translation
  t=(cm - R*cd)

    
  %% transform data to match the model
  transformeddata = R*data + repmat(t, 1, size(data,2))

  %%plot model and transformed data
  plot3(transformeddata(1,:),transformeddata(2,:),transformeddata(3,:), 'r.', model(1,:),model(2,:),model(3,:),'go')
  legend ('transformed data', 'model', 'Location','NorthEast')
    
end
  